package frc.robot.systems;

import java.util.Objects;
import java.util.function.Supplier;
import java.util.function.UnaryOperator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
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
 * each robot integration rather than baked into this reusable class.
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

	private final Config config;
	private final Supplier<Rotation3d> robotRotationSupplier;
	private final Supplier<Double> angularVelocityDegreesPerSecondSupplier;
	private final VisionConsumer visionConsumer;
	private double lastProcessedTimestampSeconds = Double.NEGATIVE_INFINITY;
	private RejectionReason lastRejectionReason = RejectionReason.NO_TAGS;

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
			drivetrain::addVisionMeasurement);
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
		this.config = Objects.requireNonNull(cameraConfig, "config");
		this.robotRotationSupplier =
			Objects.requireNonNull(rotationSupplier, "robotRotationSupplier");
		this.angularVelocityDegreesPerSecondSupplier =
			Objects.requireNonNull(yawRateSupplier, "angularVelocityDegreesPerSecondSupplier");
		this.visionConsumer = Objects.requireNonNull(poseConsumer, "visionConsumer");

		LimelightHelpers.setPipelineIndex(config.limelightName(), config.aprilTagPipeline());
		LimelightHelpers.setLEDMode_PipelineControl(config.limelightName());
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
		boolean fused = fuseIfValid(estimate);
		setCurrentState(nextState(input));
		logResult(estimate, lastRejectionReason, fused);
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
	private boolean fuseIfValid(PoseEstimate estimate) {
		lastRejectionReason = validateIncoming(estimate);
		if (lastRejectionReason != RejectionReason.NONE) {
			return false;
		}
		Pose2d pose = config.poseTransform().apply(estimate.pose);
		if (pose == null || !isFinitePose(pose)) {
			lastRejectionReason = RejectionReason.INVALID_POSE;
			return false;
		}
		visionConsumer.accept(pose, estimate.timestampSeconds, config.measurementStdDevs());
		return true;
	}

	private RejectionReason validateIncoming(PoseEstimate estimate) {
		if (estimate == null || estimate.tagCount <= 0) {
			return RejectionReason.NO_TAGS;
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

	private void logResult(PoseEstimate estimate, RejectionReason reason, boolean accepted) {
		Logger.recordOutput("Vision/State", getCurrentState().toString());
		Logger.recordOutput("Vision/Accepted", accepted);
		Logger.recordOutput("Vision/RejectionReason", reason.toString());
		if (estimate != null) {
			Logger.recordOutput("Vision/TagCount", estimate.tagCount);
			Logger.recordOutput("Vision/TimestampSeconds", estimate.timestampSeconds);
			if (estimate.pose != null) {
				Logger.recordOutput("Vision/EstimatedPose", estimate.pose);
			}
		}
	}
}
