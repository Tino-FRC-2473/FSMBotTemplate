// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Objects;
import java.util.function.Supplier;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.systems.DrivetrainFSMSystem;
import frc.robot.systems.VisionFSMSystem;

/**
 * Robot lifecycle for the Phoenix Tuner X swerve wrapper, Xbox teleop, and optional Limelight.
 * Replacing {@code frc.robot.generated.TunerConstants} with the unmodified robot-specific
 * Phoenix Tuner X output is the only hardware-specific integration step.
 */
public class Robot extends TimedRobot {
	private TeleopInput input;
	private final Field2d simulationField = new Field2d();

	private CommandSwerveDrivetrain drivetrain;
	private DrivetrainFSMSystem drivetrainFSM;
	private VisionFSMSystem visionFSM;

	@Override
	public void robotInit() {
		input = new TeleopInput();
		SmartDashboard.putData("Field", simulationField);
		initializeFromTunerConstants();
	}

	private void initializeFromTunerConstants() {
		CommandSwerveDrivetrain initializedDrivetrain = null;
		try {
			initializedDrivetrain = TunerConstants.createDrivetrain();
			DrivetrainFSMSystem.Config driveConfig = DrivetrainFSMSystem.configFromGeometry(
				TunerConstants.kSpeedAt12Volts.in(Units.MetersPerSecond),
				Constants.RobotIntegration.MAX_LINEAR_SPEED_SCALE,
				initializedDrivetrain.getMaximumModuleRadiusMeters(),
				Constants.RobotIntegration.DRIVER_DEADBAND);

			drivetrain = initializedDrivetrain;
			drivetrainFSM = new DrivetrainFSMSystem(
				drivetrain,
				driveConfig,
				DrivetrainFSMSystem::mapXboxInput);
		} catch (RuntimeException error) {
			drivetrainFSM = null;
			visionFSM = null;
			drivetrain = null;
			if (initializedDrivetrain != null) {
				try {
					initializedDrivetrain.close();
				} catch (RuntimeException closeError) {
					error.addSuppressed(closeError);
				}
			}
			reportInitializationFailure(
				"Swerve drivetrain was not initialized. Replace the TunerConstants scaffold "
					+ "with this robot's unmodified 2026 Phoenix Tuner X output and verify its generated "
					+ "speed and module geometry. Details: " + error);
			return;
		}

		try {
			visionFSM = createVisionIfEnabled(
				Constants.RobotIntegration.ENABLE_LIMELIGHT_VISION,
				() -> new VisionFSMSystem(drivetrain, getVisionConfig()));
		} catch (RuntimeException error) {
			visionFSM = null;
			reportOptionalVisionFailure(
				"Limelight vision was not initialized; drivetrain remains available. Details: " + error);
		}
	}

	/** Lazily create the camera integration so the disabled path has no NetworkTables side effects. */
	static VisionFSMSystem createVisionIfEnabled(
		boolean enabled,
		Supplier<VisionFSMSystem> visionFactory) {
		Objects.requireNonNull(visionFactory, "visionFactory");
		return enabled ? Objects.requireNonNull(visionFactory.get(), "visionFactory result") : null;
	}

	private void reportInitializationFailure(String message) {
		if (RobotBase.isReal()) {
			DriverStation.reportError(message, false);
		} else {
			DriverStation.reportWarning(message, false);
		}
	}

	private void reportOptionalVisionFailure(String message) {
		if (RobotBase.isReal()) {
			DriverStation.reportError(message, false);
		} else {
			DriverStation.reportWarning(message, false);
		}
	}

	/** Camera settings used only when {@link Constants.RobotIntegration#ENABLE_LIMELIGHT_VISION} is true. */
	protected VisionFSMSystem.Config getVisionConfig() {
		// Defaults assume the camera mounting transform is configured in Limelight Web UI.
		return VisionFSMSystem.Config.defaults(
			Constants.RobotIntegration.LIMELIGHT_NAME,
			Constants.RobotIntegration.APRILTAG_PIPELINE);
	}

	@Override
	public void autonomousInit() {
		if (drivetrainFSM != null) {
			drivetrainFSM.reset();
		}
		if (visionFSM != null) {
			visionFSM.reset();
		}
	}

	@Override
	public void autonomousPeriodic() {
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
		if (visionFSM != null) {
			visionFSM.updateVision();
		}
	}

	@Override
	public void teleopInit() {
		if (drivetrainFSM != null) {
			drivetrainFSM.reset();
		}
		if (visionFSM != null) {
			visionFSM.reset();
		}
	}

	@Override
	public void teleopPeriodic() {
		if (drivetrain != null && input.isFieldCentricResetPressed()) {
			drivetrain.seedFieldCentric();
		}
		if (visionFSM != null) {
			visionFSM.update(input);
		}
		if (drivetrainFSM != null) {
			drivetrainFSM.update(input);
		}
	}

	@Override
	public void disabledInit() {
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
	}

	@Override
	public void disabledPeriodic() {
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
		if (visionFSM != null) {
			visionFSM.updateVision();
		}
	}

	@Override
	public void testInit() {
		if (drivetrainFSM != null) {
			drivetrainFSM.reset();
		}
		if (visionFSM != null) {
			visionFSM.reset();
		}
	}

	@Override
	public void testPeriodic() {
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
		if (visionFSM != null) {
			visionFSM.updateVision();
		}
	}

	@Override
	public void simulationPeriodic() {
		if (drivetrain != null) {
			simulationField.setRobotPose(drivetrain.getPose());
		}
	}

	@Override
	public void close() {
		if (drivetrain != null) {
			drivetrain.close();
		}
		super.close();
	}
}
