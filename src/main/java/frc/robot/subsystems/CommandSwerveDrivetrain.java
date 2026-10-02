package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.util.DriveFeedforwards;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;

/**
 * Shared Phoenix 6 behavior for the standard 2026 Phoenix Tuner X generated constants.
 * Tuner X's unmodified {@code TunerConstants.java} constructs this class and supplies all
 * robot-specific device types, CAN configuration, module constants, offsets, and gains.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
	private static final double SIM_LOOP_PERIOD_SECONDS = 0.005;
	private static final Rotation2d BLUE_OPERATOR_FORWARD = Rotation2d.kZero;
	private static final Rotation2d RED_OPERATOR_FORWARD = Rotation2d.k180deg;

	private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withForwardPerspective(ForwardPerspectiveValue.OperatorPerspective);
	private final SwerveRequest.ApplyRobotSpeeds autonomousRequest =
		new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);
	private Notifier simNotifier;
	private double lastSimTimeSeconds;
	private static final double DETAILED_LOG_PERIOD_SECONDS = 0.05;
	private final String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
	private double lastDetailedLogTimestampSeconds = Double.NEGATIVE_INFINITY;
	private volatile double commandedVxMetersPerSecond;
	private volatile double commandedVyMetersPerSecond;
	private volatile double commandedOmegaRadiansPerSecond;

	public CommandSwerveDrivetrain(
		SwerveDrivetrainConstants drivetrainConstants,
		SwerveModuleConstants<?, ?, ?>... modules) {
		super(drivetrainConstants, modules);
		// Blue is the safe fallback until Driver Station alliance data arrives.
		setOperatorPerspectiveForward(BLUE_OPERATOR_FORWARD);
		registerTelemetry(this::logDrivetrainTelemetry);
		if (Utils.isSimulation()) {
			startSimulationThread();
		}
	}

	/** Apply a field-relative chassis velocity in meters/second and radians/second. */
	public void driveFieldCentric(double velocityX, double velocityY, double angularRate) {
		commandedVxMetersPerSecond = velocityX;
		commandedVyMetersPerSecond = velocityY;
		commandedOmegaRadiansPerSecond = angularRate;
		setControl(fieldCentricRequest
			.withVelocityX(velocityX)
			.withVelocityY(velocityY)
			.withRotationalRate(angularRate));
	}

	/** Measured chassis speeds in the robot frame, independent of alliance perspective. */
	public ChassisSpeeds getRobotRelativeSpeeds() {
		return getState().Speeds;
	}

	/** Apply PathPlanner velocities and per-module force feedforwards in FL, FR, BL, BR order. */
	public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards) {
		commandedVxMetersPerSecond = speeds.vxMetersPerSecond;
		commandedVyMetersPerSecond = speeds.vyMetersPerSecond;
		commandedOmegaRadiansPerSecond = speeds.omegaRadiansPerSecond;
		Logger.recordOutput("Drivetrain/Chassis/PathPlannerFeedforwardX_N",
			feedforwards.robotRelativeForcesXNewtons());
		Logger.recordOutput("Drivetrain/Chassis/PathPlannerFeedforwardY_N",
			feedforwards.robotRelativeForcesYNewtons());
		setControl(autonomousRequest
			.withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
			.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
			.withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()));
	}

	/** Command zero chassis velocity. */
	public void stop() {
		driveFieldCentric(0.0, 0.0, 0.0);
	}

	/** Log chassis state on every Phoenix odometry update and detailed hardware signals at 20 Hz. */
	private void logDrivetrainTelemetry(SwerveDriveState state) {
		Logger.recordOutput("Drivetrain/Chassis/Pose", state.Pose);
		Logger.recordOutput("Drivetrain/Chassis/PoseX_m", state.Pose.getX());
		Logger.recordOutput("Drivetrain/Chassis/PoseY_m", state.Pose.getY());
		Logger.recordOutput("Drivetrain/Chassis/Heading_deg", state.Pose.getRotation().getDegrees());
		Logger.recordOutput("Drivetrain/Chassis/RawHeading_deg", state.RawHeading.getDegrees());
		Logger.recordOutput("Drivetrain/Chassis/MeasuredVx_mps", state.Speeds.vxMetersPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/MeasuredVy_mps", state.Speeds.vyMetersPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/MeasuredOmega_radps", state.Speeds.omegaRadiansPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/CommandedVx_mps", commandedVxMetersPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/CommandedVy_mps", commandedVyMetersPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/CommandedOmega_radps", commandedOmegaRadiansPerSecond);
		Logger.recordOutput("Drivetrain/Chassis/OdometryPeriod_ms", state.OdometryPeriod * 1000.0);
		Logger.recordOutput("Drivetrain/Chassis/SuccessfulDaqs", state.SuccessfulDaqs);
		Logger.recordOutput("Drivetrain/Chassis/FailedDaqs", state.FailedDaqs);
		Logger.recordOutput("Drivetrain/Chassis/OdometryValid", isOdometryValid());
		Logger.recordOutput("Drivetrain/Chassis/BatteryVoltage", RobotController.getBatteryVoltage());

		if (state.ModuleStates == null || state.ModuleTargets == null || state.ModulePositions == null) {
			return;
		}
		for (int index = 0; index < moduleNames.length && index < state.ModuleStates.length
			&& index < state.ModuleTargets.length && index < state.ModulePositions.length; index++) {
			String modulePath = "Drivetrain/Modules/" + moduleNames[index];
			var measured = state.ModuleStates[index];
			var target = state.ModuleTargets[index];
			var position = state.ModulePositions[index];
			Logger.recordOutput(modulePath + "/MeasuredSpeed_mps", measured.speedMetersPerSecond);
			Logger.recordOutput(modulePath + "/MeasuredAngle_deg", measured.angle.getDegrees());
			Logger.recordOutput(modulePath + "/TargetSpeed_mps", target.speedMetersPerSecond);
			Logger.recordOutput(modulePath + "/TargetAngle_deg", target.angle.getDegrees());
			Logger.recordOutput(modulePath + "/DriveDistance_m", position.distanceMeters);
			Logger.recordOutput(modulePath + "/ModulePositionAngle_deg", position.angle.getDegrees());
		}

		if (state.Timestamp - lastDetailedLogTimestampSeconds < DETAILED_LOG_PERIOD_SECONDS) {
			return;
		}
		lastDetailedLogTimestampSeconds = state.Timestamp;
		logGyroTelemetry();
		SwerveModule<TalonFX, TalonFX, CANcoder>[] modules = getModules();
		for (int index = 0; index < modules.length && index < moduleNames.length; index++) {
			logModuleHardware(moduleNames[index], modules[index]);
		}
	}

	private void logModuleHardware(String name, SwerveModule<TalonFX, TalonFX, CANcoder> module) {
		String root = "Drivetrain/Modules/" + name;
		logMotor(root + "/DriveMotor", module.getDriveMotor());
		logMotor(root + "/SteerMotor", module.getSteerMotor());

		CANcoder encoder = module.getEncoder();
		String encoderRoot = root + "/SteerEncoder";
		Logger.recordOutput(encoderRoot + "/Connected", encoder.isConnected());
		logSignal(encoderRoot + "/AbsolutePosition", encoder.getAbsolutePosition());
		logSignal(encoderRoot + "/Position", encoder.getPosition());
		logSignal(encoderRoot + "/Velocity", encoder.getVelocity());
		logSignal(encoderRoot + "/SupplyVoltage", encoder.getSupplyVoltage());
		Logger.recordOutput(encoderRoot + "/MagnetHealth", encoder.getMagnetHealth().getValue().toString());
		Logger.recordOutput(encoderRoot + "/Faults", encoder.getFaultField().getValue().toString());
		Logger.recordOutput(encoderRoot + "/StickyFaults", encoder.getStickyFaultField().getValue().toString());
	}

	private void logMotor(String root, TalonFX motor) {
		Logger.recordOutput(root + "/Connected", motor.isConnected());
		logSignal(root + "/Position", motor.getPosition());
		logSignal(root + "/Velocity", motor.getVelocity());
		logSignal(root + "/Acceleration", motor.getAcceleration());
		logSignal(root + "/RotorPosition", motor.getRotorPosition());
		logSignal(root + "/RotorVelocity", motor.getRotorVelocity());
		logSignal(root + "/MotorVoltage", motor.getMotorVoltage());
		logSignal(root + "/SupplyVoltage", motor.getSupplyVoltage());
		logSignal(root + "/SupplyCurrent", motor.getSupplyCurrent());
		logSignal(root + "/StatorCurrent", motor.getStatorCurrent());
		logSignal(root + "/DutyCycle", motor.getDutyCycle());
		logSignal(root + "/DeviceTemperature", motor.getDeviceTemp());
		logSignal(root + "/ProcessorTemperature", motor.getProcessorTemp());
		logSignal(root + "/ClosedLoopReference", motor.getClosedLoopReference());
		logSignal(root + "/ClosedLoopError", motor.getClosedLoopError());
		logSignal(root + "/ClosedLoopOutput", motor.getClosedLoopOutput());
		logSignal(root + "/ClosedLoopFeedForward", motor.getClosedLoopFeedForward());
		Logger.recordOutput(root + "/ControlMode", motor.getControlMode().getValue().toString());
		Logger.recordOutput(root + "/Faults", motor.getFaultField().getValue().toString());
		Logger.recordOutput(root + "/StickyFaults", motor.getStickyFaultField().getValue().toString());
	}

	private void logGyroTelemetry() {
		Pigeon2 gyro = getPigeon2();
		String root = "Drivetrain/Gyro";
		Rotation3d rotation = gyro.getRotation3d();
		Logger.recordOutput(root + "/Connected", gyro.isConnected());
		Logger.recordOutput(root + "/Rotation", rotation);
		Logger.recordOutput(root + "/Yaw_deg", rotation.getZ() * 180.0 / Math.PI);
		Logger.recordOutput(root + "/Pitch_deg", rotation.getY() * 180.0 / Math.PI);
		Logger.recordOutput(root + "/Roll_deg", rotation.getX() * 180.0 / Math.PI);
		logSignal(root + "/YawSignal", gyro.getYaw());
		logSignal(root + "/PitchSignal", gyro.getPitch());
		logSignal(root + "/RollSignal", gyro.getRoll());
		logSignal(root + "/AngularVelocityX", gyro.getAngularVelocityXDevice());
		logSignal(root + "/AngularVelocityY", gyro.getAngularVelocityYDevice());
		logSignal(root + "/AngularVelocityZ", gyro.getAngularVelocityZDevice());
		logSignal(root + "/AccelerationX", gyro.getAccelerationX());
		logSignal(root + "/AccelerationY", gyro.getAccelerationY());
		logSignal(root + "/AccelerationZ", gyro.getAccelerationZ());
		logSignal(root + "/SupplyVoltage", gyro.getSupplyVoltage());
		logSignal(root + "/Temperature", gyro.getTemperature());
		Logger.recordOutput(root + "/Faults", gyro.getFaultField().getValue().toString());
		Logger.recordOutput(root + "/StickyFaults", gyro.getStickyFaultField().getValue().toString());
	}

	private static void logSignal(String key, StatusSignal<?> signal) {
		String leafName = key.substring(key.lastIndexOf('/') + 1);
		String parentPath = key.substring(0, key.lastIndexOf('/'));
		Logger.recordOutput(key + " (" + signal.getUnits() + ")", signal.getValueAsDouble());
		Logger.recordOutput(parentPath + "/SignalStatus/" + leafName, signal.getStatus().toString());
	}

	/** Current fused field pose. */
	public Pose2d getPose() {
		return getState().Pose;
	}

	/** Field-aligned yaw plus Pigeon roll/pitch for Limelight MegaTag2 input. */
	public Rotation3d getRobotRotation3d() {
		Rotation3d pigeonRotation = getPigeon2().getRotation3d();
		// resetPose changes estimator heading, not necessarily the Pigeon's raw yaw.
		return new Rotation3d(pigeonRotation.getX(), pigeonRotation.getY(),
			getPose().getRotation().getRadians());
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
