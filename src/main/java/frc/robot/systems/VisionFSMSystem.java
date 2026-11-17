package frc.robot.systems;

import java.util.Objects;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionDefaults;
import frc.robot.input.Input;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.imported.LimelightHelpers;
import frc.robot.imported.LimelightHelpers.PoseEstimate;
import frc.robot.imported.LimelightHelpers.RawFiducial;
/**
 * Generic Limelight MegaTag2 localization FSM. Camera name, pipeline, mounting transform,
 * pose source, field bounds, quality thresholds, and measurement noise are injected by
 * each robot integration rather than baked into this reusable class. The first trustworthy
 * fix seeds the drivetrain pose; every later frame fuses normally. Translation comes from
 * MegaTag2, heading from the tag-only solve because MegaTag2 echoes the supplied gyro.
 */
public class VisionFSMSystem extends FSMSystem<VisionFSMSystem.VisionState> {
	public enum VisionState {
		WAITING_FOR_TAGS,
		TRACKING,
		REJECTING_ESTIMATE
	}

	public enum RejectionReason {
		NONE,
		NO_TAGS,
		DETECTED_NO_POSE,
		INVALID_FIDUCIALS,
		INVALID_POSE,
		INVALID_TIMESTAMP,
		STALE,
		FUTURE_TIMESTAMP,
		DUPLICATE_TIMESTAMP,
		OUT_OF_FIELD,
		HIGH_AMBIGUITY,
		TOO_FAR
	}

	/**
	 * Optional inclusive field coordinate bounds, in meters.
	 * @param minX minimum field x, in meters
	 * @param maxX maximum field x, in meters
	 * @param minY minimum field y, in meters
	 * @param maxY maximum field y, in meters
	 */
	public record FieldBounds(double minX, double maxX, double minY, double maxY) {
		/** Reject nonfinite or inverted bounds. */
		public FieldBounds {
			if (!Double.isFinite(minX) || !Double.isFinite(maxX)
				|| !Double.isFinite(minY) || !Double.isFinite(maxY)
				|| minX >= maxX || minY >= maxY) {
				throw new IllegalArgumentException("Field bounds must be finite with min < max");
			}
		}

		boolean contains(Pose2d pose) {
			return pose.getX() >= minX && pose.getX() <= maxX
				&& pose.getY() >= minY && pose.getY() <= maxY;
		}
	}

	/**
	 * Camera pose, quality gates, and measurement uncertainty for one Limelight.
	 * @param limelightName NetworkTables name of the camera
	 * @param aprilTagPipeline AprilTag pipeline index
	 * @param configureCameraPose whether to push the mounting transform to the camera
	 * @param robotToCamera camera mounting transform, used only when configuring the pose
	 * @param maxMeasurementAgeSeconds oldest acceptable measurement, in seconds
	 * @param maxFutureTimestampSeconds largest tolerated future timestamp offset, in seconds
	 * @param maxTagAmbiguity highest acceptable per-tag ambiguity
	 * @param maxAverageTagDistanceMeters farthest acceptable average tag distance
	 * @param fieldBounds optional inclusive field bounds, or null to skip the check
	 * @param measurementStdDevs noise passed to the pose estimator
	 * @param poseTransform optional transform applied to accepted poses
	 */
	public record Config(
		String limelightName,
		int aprilTagPipeline,
		boolean configureCameraPose,
		Transform3d robotToCamera,
		double maxMeasurementAgeSeconds,
		double maxFutureTimestampSeconds,
		double maxTagAmbiguity,
		double maxAverageTagDistanceMeters,
		FieldBounds fieldBounds,
		Matrix<N3, N1> measurementStdDevs,
		UnaryOperator<Pose2d> poseTransform) {
		/** Validate camera identification, required objects, and quality limits. */
		public Config {
			if (limelightName == null || limelightName.isBlank()) {
				throw new IllegalArgumentException("Limelight name must not be blank");
			}
			if (aprilTagPipeline < 0) {
				throw new IllegalArgumentException("AprilTag pipeline must be nonnegative");
			}
			if (robotToCamera == null || measurementStdDevs == null || poseTransform == null) {
				throw new IllegalArgumentException(
					"Camera transform, standard deviations, and pose transform are required");
			}
			if (!Double.isFinite(maxMeasurementAgeSeconds) || maxMeasurementAgeSeconds < 0.0
				|| !Double.isFinite(maxFutureTimestampSeconds) || maxFutureTimestampSeconds < 0.0
				|| !Double.isFinite(maxTagAmbiguity) || maxTagAmbiguity < 0.0
				|| !Double.isFinite(maxAverageTagDistanceMeters)
				|| maxAverageTagDistanceMeters <= 0.0) {
				throw new IllegalArgumentException(
					"Vision quality limits must be finite and nonnegative");
			}
		}

		/**
		 * Build a configuration with template defaults and no field bounds.
		 * @param limelightName NetworkTables name of the camera
		 * @param pipeline AprilTag pipeline index
		 * @return configuration using the generic vision defaults
		 */
		public static Config defaults(String limelightName, int pipeline) {
			return new Config(
				limelightName,
				pipeline,
				false,
				new Transform3d(),
				VisionDefaults.MAX_MEASUREMENT_AGE_SECONDS,
				VisionDefaults.MAX_FUTURE_TIMESTAMP_SECONDS,
				VisionDefaults.MAX_TAG_AMBIGUITY,
				VisionDefaults.MAX_AVERAGE_TAG_DISTANCE_METERS,
				null,
				VisionDefaults.MEGATAG2_STD_DEVS,
				UnaryOperator.identity());
		}
	}

	/** Consumer lets this FSM fuse through the robot's drivetrain or pose estimator. */
	@FunctionalInterface
	public interface VisionConsumer {
		/**
		 * Consume one accepted field pose.
		 * @param pose field-relative pose to fuse
		 * @param timestampSeconds FPGA timestamp of the camera frame
		 * @param standardDeviations measurement noise for the pose estimator
		 */
		void accept(Pose2d pose, double timestampSeconds, Matrix<N3, N1> standardDeviations);
	}

	/** Seeder lets this FSM place the drivetrain pose at the first trustworthy vision fix. */
	@FunctionalInterface
	public interface VisionSeeder {
		/**
		 * Adopt one field pose as the estimator's new starting point.
		 * @param pose field-relative pose to seed
		 */
		void seed(Pose2d pose);
	}

	private final Config config;
	private final Supplier<Rotation3d> robotRotationSupplier;
	private final Supplier<Double> angularVelocityDegreesPerSecondSupplier;
	private final VisionConsumer visionConsumer;
	private final VisionSeeder poseSeeder;
	private final Matrix<N3, N1> headingStdDevs;
	private double lastProcessedTimestampSeconds = Double.NEGATIVE_INFINITY;
	private RejectionReason lastRejectionReason = RejectionReason.NO_TAGS;
	private boolean localized;
	private boolean lastHeadingFromTags;
	private int seedCount;

	/**
	 * Create a vision FSM bound to a drivetrain's heading, yaw rate, and pose estimator.
	 * @param drivetrain source of gyro heading and consumer of accepted measurements
	 * @param cameraConfig camera name, quality gates, and measurement noise
	 */
	public VisionFSMSystem(CommandSwerveDrivetrain drivetrain, Config cameraConfig) {
		this(
			cameraConfig,
			drivetrain::getRobotRotation3d,
			() -> Math.toDegrees(drivetrain.getAngularVelocityRadiansPerSecond()),
			drivetrain::addVisionMeasurement,
			drivetrain::seedPoseFromVision);
	}

	/**
	 * Create a vision FSM with injected estimator seams so vision can be tested without CAN.
	 * @param cameraConfig camera name, quality gates, and measurement noise
	 * @param rotationSupplier field-aligned robot rotation published for MegaTag2
	 * @param yawRateSupplier measured yaw rate in degrees per second
	 * @param poseConsumer consumer for accepted poses and their standard deviations
	 */
	public VisionFSMSystem(
		Config cameraConfig,
		Supplier<Rotation3d> rotationSupplier,
		Supplier<Double> yawRateSupplier,
		VisionConsumer poseConsumer) {
		this(cameraConfig, rotationSupplier, yawRateSupplier, poseConsumer, pose -> { });
	}

	/**
	 * Create a vision FSM with injected localization seams so seeding is testable without CAN.
	 * @param cameraConfig camera name, quality gates, and measurement noise
	 * @param rotationSupplier field-aligned robot rotation published for MegaTag2
	 * @param yawRateSupplier measured yaw rate in degrees per second
	 * @param poseConsumer consumer for accepted poses and their standard deviations
	 * @param seeder replaces the estimator pose with the first trustworthy fix
	 */
	public VisionFSMSystem(
		Config cameraConfig,
		Supplier<Rotation3d> rotationSupplier,
		Supplier<Double> yawRateSupplier,
		VisionConsumer poseConsumer,
		VisionSeeder seeder) {
		this.config = Objects.requireNonNull(cameraConfig, "config");
		this.robotRotationSupplier =
			Objects.requireNonNull(rotationSupplier, "robotRotationSupplier");
		this.angularVelocityDegreesPerSecondSupplier =
			Objects.requireNonNull(yawRateSupplier, "angularVelocityDegreesPerSecondSupplier");
		this.visionConsumer = Objects.requireNonNull(poseConsumer, "visionConsumer");
		this.poseSeeder = Objects.requireNonNull(seeder, "seeder");
		this.headingStdDevs = VecBuilder.fill(
			this.config.measurementStdDevs().get(0, 0),
			this.config.measurementStdDevs().get(1, 0),
			VisionDefaults.HEADING_STD_DEV_RADIANS);

		LimelightHelpers.setPipelineIndex(config.limelightName(), config.aprilTagPipeline());
		LimelightHelpers.setLEDMode_PipelineControl(config.limelightName());
		LimelightHelpers.SetIMUMode(config.limelightName(), 3);
		LimelightHelpers.SetIMUAssistAlpha(config.limelightName(), 0.01);
		if (config.configureCameraPose()) {
			Transform3d camera = config.robotToCamera();
			LimelightHelpers.setCameraPose_RobotSpace(
				config.limelightName(),
				camera.getX(), camera.getY(), camera.getZ(),
				Math.toDegrees(camera.getRotation().getX()),
				Math.toDegrees(camera.getRotation().getY()),
				Math.toDegrees(camera.getRotation().getZ()));
		}
		reset();
	}

	@Override
	public void reset() {
		// Preserve the last timestamp across mode changes so a frame is only fused once.
		// Localization is preserved for the same reason: a seeded pose must outlive a mode change.
		lastRejectionReason = RejectionReason.NO_TAGS;
		setCurrentState(VisionState.WAITING_FOR_TAGS);
	}

	@Override
	public void update(Input input) {
		Rotation3d robotRotation = robotRotationSupplier.get();
		LimelightHelpers.SetRobotOrientation_NoFlush(
			config.limelightName(),
			Math.toDegrees(robotRotation.getZ()),
			angularVelocityDegreesPerSecondSupplier.get(),
			Math.toDegrees(robotRotation.getY()), 0.0,
			Math.toDegrees(robotRotation.getX()), 0.0);

		PoseEstimate estimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(config.limelightName());
		// MegaTag2 reports the heading it was told, so the tag-only solve from the same frame
		// is the only measurement that can actually correct yaw.
		PoseEstimate tagPoseEstimate =
			LimelightHelpers.getBotPoseEstimate_wpiBlue(config.limelightName());
		// Distinguishes "the pipeline sees no tag" from "a tag is visible but MegaTag2
		// produced no localization pose" (missing field map or robot-space pose).
		boolean cameraHasTarget = LimelightHelpers.getTV(config.limelightName());
		boolean fused = fuseIfValid(estimate, tagPoseEstimate, cameraHasTarget);
		setCurrentState(nextState(input));
		logResult(estimate, lastRejectionReason, fused, cameraHasTarget);
	}

	@Override
	protected VisionState nextState(Input input) {
		return switch (lastRejectionReason) {
			case NONE -> VisionState.TRACKING;
			case NO_TAGS -> VisionState.WAITING_FOR_TAGS;
			default -> VisionState.REJECTING_ESTIMATE;
		};
	}

	/**
	 * Most recent rejection reason.
	 * @return the last reason, or {@link RejectionReason#NONE} after a fused measurement
	 */
	public RejectionReason getLastRejectionReason() {
		return lastRejectionReason;
	}

	// A rejected frame still consumes its timestamp so the same frame is never retried.
	private boolean fuseIfValid(
		PoseEstimate estimate, PoseEstimate tagPoseEstimate, boolean cameraHasTarget) {
		lastHeadingFromTags = false;
		lastRejectionReason = validateIncoming(estimate, cameraHasTarget);
		if (lastRejectionReason != RejectionReason.NONE) {
			return false;
		}
		Pose2d pose = config.poseTransform().apply(
			poseWithTrustedHeading(estimate, tagPoseEstimate));
		if (pose == null || !isFinitePose(pose)) {
			lastRejectionReason = RejectionReason.INVALID_POSE;
			return false;
		}
		if (!localized && isSeedQualitySufficient(estimate)) {
			// Filtering alone leaves the estimator's odometry track at the boot pose, so any
			// discarded correction reveals (0,0). Adopting the first trustworthy fix moves
			// odometry itself into field coordinates.
			poseSeeder.seed(pose);
			localized = true;
			seedCount++;
			return true;
		}
		visionConsumer.accept(pose, fuseTimestamp(estimate.timestampSeconds),
			lastHeadingFromTags ? headingStdDevs : config.measurementStdDevs());
		return true;
	}

	// MegaTag2 solves its position from the heading this robot publishes, so its theta carries
	// no new information. Substitute the tag-only rotation when that fix is trustworthy; keep
	// the gyro heading, and its huge theta, otherwise.
	private Pose2d poseWithTrustedHeading(PoseEstimate estimate, PoseEstimate tagPoseEstimate) {
		if (!isHeadingMeasurementUsable(tagPoseEstimate, Timer.getFPGATimestamp(), config)) {
			return estimate.pose;
		}
		lastHeadingFromTags = true;
		return new Pose2d(
			estimate.pose.getTranslation(), tagPoseEstimate.pose.getRotation());
	}

	// The estimator silently drops a measurement newer than its newest odometry sample, which
	// would discard a frame this validator already accepted.
	private static double fuseTimestamp(double timestampSeconds) {
		return Math.min(
			timestampSeconds, Timer.getFPGATimestamp() - VisionDefaults.FUSE_MARGIN_SECONDS);
	}

	private RejectionReason validateIncoming(PoseEstimate estimate, boolean cameraHasTarget) {
		if (estimate == null || estimate.tagCount <= 0) {
			return cameraHasTarget ? RejectionReason.DETECTED_NO_POSE : RejectionReason.NO_TAGS;
		}
		if (!Double.isFinite(estimate.timestampSeconds) || estimate.timestampSeconds <= 0.0) {
			return RejectionReason.INVALID_TIMESTAMP;
		}
		if (!isTimestampNew(estimate.timestampSeconds, lastProcessedTimestampSeconds)) {
			return RejectionReason.DUPLICATE_TIMESTAMP;
		}
		lastProcessedTimestampSeconds = estimate.timestampSeconds;
		return validateEstimate(estimate, Timer.getFPGATimestamp(), config);
	}

	/**
	 * Pure validation function for testing and alternate integrations.
	 * @param estimate pose estimate to validate
	 * @param nowSeconds current FPGA timestamp, in seconds
	 * @param config quality gates to apply
	 * @return the rejection reason, or {@link RejectionReason#NONE} when acceptable
	 */
	public static RejectionReason validateEstimate(
		PoseEstimate estimate, double nowSeconds, Config config) {
		Objects.requireNonNull(config, "config");
		if (estimate == null || estimate.tagCount <= 0) {
			return RejectionReason.NO_TAGS;
		}
		if (estimate.rawFiducials == null || estimate.rawFiducials.length != estimate.tagCount) {
			return RejectionReason.INVALID_FIDUCIALS;
		}
		if (estimate.pose == null
			|| !Double.isFinite(estimate.pose.getX())
			|| !Double.isFinite(estimate.pose.getY())
			|| !Double.isFinite(estimate.pose.getRotation().getRadians())) {
			return RejectionReason.INVALID_POSE;
		}
		if (!Double.isFinite(estimate.timestampSeconds) || estimate.timestampSeconds <= 0.0
			|| !Double.isFinite(nowSeconds)) {
			return RejectionReason.INVALID_TIMESTAMP;
		}
		if (nowSeconds - estimate.timestampSeconds > config.maxMeasurementAgeSeconds()) {
			return RejectionReason.STALE;
		}
		if (estimate.timestampSeconds - nowSeconds > config.maxFutureTimestampSeconds()) {
			return RejectionReason.FUTURE_TIMESTAMP;
		}
		if (config.fieldBounds() != null && !config.fieldBounds().contains(estimate.pose)) {
			return RejectionReason.OUT_OF_FIELD;
		}
		if (!Double.isFinite(estimate.avgTagDist) || estimate.avgTagDist < 0.0
			|| estimate.avgTagDist > config.maxAverageTagDistanceMeters()) {
			return RejectionReason.TOO_FAR;
		}

		boolean hasAcceptableAmbiguity = false;
		for (RawFiducial fiducial : estimate.rawFiducials) {
			if (fiducial == null || !Double.isFinite(fiducial.ambiguity)) {
				return RejectionReason.INVALID_FIDUCIALS;
			}
			if (fiducial.ambiguity >= 0.0 && fiducial.ambiguity <= config.maxTagAmbiguity()) {
				hasAcceptableAmbiguity = true;
			}
		}
		return hasAcceptableAmbiguity ? RejectionReason.NONE : RejectionReason.HIGH_AMBIGUITY;
	}

	/**
	 * Whether a validated fix is trustworthy enough to replace the estimator's boot pose.
	 * @param estimate estimate that already passed validation
	 * @return true when several tags were seen, or one tag inside the seed distance limit
	 */
	public static boolean isSeedQualitySufficient(PoseEstimate estimate) {
		Objects.requireNonNull(estimate, "estimate");
		// Several tags are always well constrained; a lone tag only when it is close enough.
		return estimate.tagCount >= VisionDefaults.MIN_SEED_TAGS
			&& (estimate.tagCount > 1 || (Double.isFinite(estimate.avgTagDist)
				&& estimate.avgTagDist <= VisionDefaults.MAX_SEED_TAG_DISTANCE_METERS));
	}

	/**
	 * Whether the tag-only pose may correct the estimator's heading.
	 * @param tagPoseEstimate tag-only (MegaTag1) estimate for the same camera frame
	 * @param nowSeconds current FPGA timestamp, in seconds
	 * @param config quality gates to apply
	 * @return true when the fix is fresh, unambiguous, and of seed quality
	 */
	public static boolean isHeadingMeasurementUsable(
		PoseEstimate tagPoseEstimate, double nowSeconds, Config config) {
		return tagPoseEstimate != null
			&& isSeedQualitySufficient(tagPoseEstimate)
			&& validateEstimate(tagPoseEstimate, nowSeconds, config) == RejectionReason.NONE;
	}

	private static boolean isFinitePose(Pose2d pose) {
		return Double.isFinite(pose.getX())
			&& Double.isFinite(pose.getY())
			&& Double.isFinite(pose.getRotation().getRadians());
	}

	/**
	 * Whether a camera timestamp is usable and newer than the last fused frame.
	 * @param timestampSeconds timestamp of the incoming estimate
	 * @param previousTimestampSeconds timestamp of the last fused estimate
	 * @return true when the estimate is finite, positive, and strictly newer
	 */
	public static boolean isTimestampNew(double timestampSeconds, double previousTimestampSeconds) {
		return Double.isFinite(timestampSeconds)
			&& timestampSeconds > 0.0
			&& timestampSeconds > previousTimestampSeconds;
	}

	private void logResult(
		PoseEstimate estimate, RejectionReason reason, boolean accepted, boolean cameraHasTarget) {
		Logger.recordOutput("Vision/State", getCurrentState().toString());
		Logger.recordOutput("Vision/Accepted", accepted);
		Logger.recordOutput("Vision/RejectionReason", reason.toString());
		Logger.recordOutput("Vision/Localized", localized);
		Logger.recordOutput("Vision/SeedCount", seedCount);
		Logger.recordOutput("Vision/HeadingFromTags", lastHeadingFromTags);
		if (Double.isFinite(lastProcessedTimestampSeconds)) {
			Logger.recordOutput(
				"Vision/FusedTimestampAge_s",
				Timer.getFPGATimestamp() - lastProcessedTimestampSeconds);
		}
		Logger.recordOutput("Vision/Camera/HasTarget", cameraHasTarget);
		Logger.recordOutput(
			"Vision/Camera/TargetID", LimelightHelpers.getFiducialID(config.limelightName()));
		Logger.recordOutput(
			"Vision/Camera/Heartbeat", LimelightHelpers.getHeartbeat(config.limelightName()));
		Logger.recordOutput(
			"Vision/Camera/Pipeline",
			LimelightHelpers.getCurrentPipelineIndex(config.limelightName()));
		if (estimate != null) {
			Logger.recordOutput("Vision/TagCount", estimate.tagCount);
			Logger.recordOutput("Vision/TimestampSeconds", estimate.timestampSeconds);
			if (estimate.pose != null && estimate.tagCount > 0) {
				Logger.recordOutput("Vision/EstimatedPose", estimate.pose);
			}
		}
	}
}
