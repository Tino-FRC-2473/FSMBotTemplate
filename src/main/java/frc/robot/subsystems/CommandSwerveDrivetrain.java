package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Shared Phoenix 6 behavior for the standard 2026 Phoenix Tuner X generated constants.
 * Tuner X's unmodified {@code TunerConstants.java} constructs this class and supplies all
 * robot-specific device types, CAN configuration, module constants, offsets, and gains.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain {
	private static final double SIM_LOOP_PERIOD_SECONDS = 0.005;
	private static final Rotation2d BLUE_OPERATOR_FORWARD = Rotation2d.kZero;
	private static final Rotation2d RED_OPERATOR_FORWARD = Rotation2d.k180deg;

	private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
	private Notifier simNotifier;
	private double lastSimTimeSeconds;

	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, modules);
		// Blue is the safe fallback until Driver Station alliance data arrives.
		setOperatorPerspectiveForward(BLUE_OPERATOR_FORWARD);
		if (Utils.isSimulation()) {
			startSimulationThread();
		}
	}

	/** Apply a field-relative chassis velocity in meters/second and radians/second. */
	public void driveFieldCentric(double velocityX, double velocityY, double angularRate) {
		setControl(fieldCentricRequest
			.withVelocityX(velocityX)
			.withVelocityY(velocityY)
			.withRotationalRate(angularRate));
	}

	/** Command zero chassis velocity. */
	public void stop() {
		driveFieldCentric(0.0, 0.0, 0.0);
	}

	/** Current fused field pose. */
	public Pose2d getPose() {
		return getState().Pose;
	}

	/** Current Pigeon orientation for Limelight MegaTag2 input. */
	public Rotation3d getRobotRotation3d() {
		return getPigeon2().getRotation3d();
	}

	/** Current measured robot angular velocity in radians/second. */
	public double getAngularVelocityRadiansPerSecond() {
		return getState().Speeds.omegaRadiansPerSecond;
	}

	/** Maximum radius of the generated module locations from the robot center. */
	public double getMaximumModuleRadiusMeters() {
		double maxRadius = 0.0;
		for (Translation2d location : getModuleLocations()) {
			maxRadius = Math.max(maxRadius, location.getNorm());
		}
		return maxRadius;
	}

	/**
	 * Keep alliance-relative forward synchronized only while disabled. Missing alliance data
	 * leaves the explicit blue-forward default in place, avoiding a perspective jump while enabled.
	 */
	public void updateOperatorPerspective() {
		if (!DriverStation.isDisabled()) {
			return;
		}
		DriverStation.getAlliance().ifPresent(alliance -> setOperatorPerspectiveForward(
			alliance == DriverStation.Alliance.Red
				? RED_OPERATOR_FORWARD
				: BLUE_OPERATOR_FORWARD));
	}

	/**
	 * Phoenix stores drivetrain measurements on its own clock; translate FPGA timestamps
	 * provided by Limelight to the current Phoenix time base exactly once here.
	 */
	@Override
	public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
		super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
	}

	@Override
	public void addVisionMeasurement(
		Pose2d visionRobotPoseMeters,
		double timestampSeconds,
		Matrix<N3, N1> visionMeasurementStdDevs) {
		super.addVisionMeasurement(
			visionRobotPoseMeters,
			Utils.fpgaToCurrentTime(timestampSeconds),
			visionMeasurementStdDevs);
	}

	/** Stop the desktop simulation notifier when shutting down the robot application. */
	@Override
	public void close() {
		if (simNotifier != null) {
			simNotifier.close();
			simNotifier = null;
		}
		super.close();
	}

	private void startSimulationThread() {
		lastSimTimeSeconds = Utils.getCurrentTimeSeconds();
		simNotifier = new Notifier(() -> {
			double currentTimeSeconds = Utils.getCurrentTimeSeconds();
			double deltaTimeSeconds = currentTimeSeconds - lastSimTimeSeconds;
			lastSimTimeSeconds = currentTimeSeconds;
			if (Double.isFinite(deltaTimeSeconds) && deltaTimeSeconds > 0.0) {
				updateSimState(deltaTimeSeconds, RobotController.getBatteryVoltage());
			}
		});
		simNotifier.setName("Phoenix Swerve Simulation");
		simNotifier.startPeriodic(SIM_LOOP_PERIOD_SECONDS);
	}
}
