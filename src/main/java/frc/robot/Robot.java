// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.littletonrobotics.junction.LoggedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.input.TeleopInput;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.systems.DrivetrainFSMSystem;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.PathPlannerAutoSystem;
import frc.robot.systems.VisionFSMSystem;

/**
 * Robot lifecycle for Phoenix swerve, Xbox teleop, PathPlanner auto, and optional Limelight.
 * Drivetrain hardware comes from unmodified robot-specific Phoenix Tuner X output;
 * autonomous physics and routines come from the PathPlanner GUI.
 */
public class Robot extends LoggedRobot {
	private static boolean loggingConfigured;

	private TeleopInput input;
	private final Field2d field = new Field2d();

	private CommandSwerveDrivetrain drivetrain;
	private DrivetrainFSMSystem drivetrainFSM;
	private VisionFSMSystem visionFSM;
	private PathPlannerAutoSystem autonomousSystem;
	private final List<FSMSystem<?>> mechanisms = new ArrayList<>();
	private boolean mechanismRegistrationClosed;
	private boolean mechanismFaulted;
	private boolean mechanismControllerWasConnected;

	public Robot() {
		configureLogging();
	}

	/** Start AdvantageKit once before any robot devices are initialized. */
	private static synchronized void configureLogging() {
		if (loggingConfigured) {
			return;
		}
		Logger.recordMetadata("Project", "FRC Swerve Robot");
		if (RobotBase.isReal()) {
			Logger.addDataReceiver(new WPILOGWriter());
		}
		Logger.addDataReceiver(new NT4Publisher());
		Logger.start();
		loggingConfigured = true;
	}

	// Hardware-free lifecycle tests can provide a chooser and cleanup callbacks.
	Robot(PathPlannerAutoSystem autonomousSystem, TeleopInput input) {
		this.autonomousSystem = autonomousSystem;
		this.input = input;
	}

	@Override
	public void robotInit() {
		input = new TeleopInput();
		SmartDashboard.putData("Field", field);
		SendableChooser<Command> unavailableChooser = new SendableChooser<>();
		unavailableChooser.setDefaultOption("None", Commands.none());
		SmartDashboard.putData("Auto Chooser", unavailableChooser);
		SmartDashboard.putBoolean("Auto/Ready", false);
		try {
			initializeMechanisms();
		} catch (RuntimeException error) {
			mechanismFaulted = true;
			reportInitializationFailure("Mechanism initialization failed: " + error);
			cancelAutonomous();
		} finally {
			mechanismRegistrationClosed = true;
		}
		initializeFromTunerConstants();
	}

	/**
	 * Construct/register robot-specific mechanisms and PathPlanner NamedCommands here.
	 * Called before autos are loaded; intentionally empty so example hardware is not constructed.
	 */
	protected void initializeMechanisms() { }

	/** Add a shared mechanism once; drivetrain and vision keep their dedicated lifecycle. */
	protected final void registerMechanism(FSMSystem<?> mechanism) {
		Objects.requireNonNull(mechanism, "mechanism");
		if (mechanismRegistrationClosed) {
			throw new IllegalStateException("Register mechanisms before loading autonomous routines");
		}
		if (mechanism instanceof DrivetrainFSMSystem || mechanism instanceof VisionFSMSystem) {
			throw new IllegalArgumentException("Drivetrain and vision are not shared mechanisms");
		}
		if (mechanisms.stream().anyMatch(existing -> existing == mechanism)) {
			throw new IllegalArgumentException("Mechanism instance is already registered");
		}
		mechanisms.add(mechanism);
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
			autonomousSystem = PathPlannerAutoSystem.create(drivetrain, this::resetMechanisms);
			SmartDashboard.putData("Auto Chooser", autonomousSystem.getChooser());
			SmartDashboard.putBoolean("Auto/Ready", true);
		} catch (Exception error) {
			autonomousSystem = null;
			reportInitializationFailure(
				"PathPlanner autonomous is unavailable; teleop remains available. Configure "
					+ "pathplanner/settings.json and autos in the deploy directory. Details: " + error);
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

	/**
	 * Camera settings used only when
	 * {@link Constants.RobotIntegration#ENABLE_LIMELIGHT_VISION} is true.
	 */
	protected VisionFSMSystem.Config getVisionConfig() {
		// Defaults assume the camera mounting transform is configured in Limelight Web UI.
		return VisionFSMSystem.Config.defaults(
			Constants.RobotIntegration.LIMELIGHT_NAME,
			Constants.RobotIntegration.APRILTAG_PIPELINE);
	}

	@Override
	public void autonomousInit() {
		cancelAutonomous();
		if (drivetrainFSM != null) {
			drivetrainFSM.reset();
		}
		if (visionFSM != null) {
			visionFSM.reset();
		}
		if (autonomousSystem != null && !mechanismFaulted) {
			try {
				autonomousSystem.start();
			} catch (RuntimeException error) {
				cancelAutonomous();
				reportInitializationFailure("PathPlanner auto failed to start: " + error);
			}
		}
	}

	@Override
	public void autonomousPeriodic() {
		if (autonomousSystem != null) {
			autonomousSystem.update();
			if (autonomousSystem.isRunning()) {
				updateMechanisms(null);
			}
		} else if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
		updateVisionSafely();
	}

	@Override
	public void teleopInit() {
		cancelAutonomous();
		input.synchronize();
		mechanismControllerWasConnected = input.isMechanismControllerConnected();
		if (drivetrainFSM != null) {
			drivetrainFSM.reset();
		}
		if (visionFSM != null) {
			visionFSM.reset();
		}
	}

	@Override
	public void teleopPeriodic() {
		input.poll();
		if (drivetrain != null && input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED)) {
			drivetrain.seedFieldCentric();
		}
		if (drivetrainFSM != null) {
			drivetrainFSM.update(input);
		}
		boolean mechanismConnected = input.isMechanismControllerConnected();
		if (mechanismConnected) {
			updateMechanisms(input);
		} else if (mechanismControllerWasConnected) {
			try {
				resetMechanisms();
			} catch (RuntimeException error) {
				reportInitializationFailure("Mechanism controller lost; reset failed: " + error);
			}
		}
		mechanismControllerWasConnected = mechanismConnected;
		updateVisionSafely();
	}

	@Override
	public void disabledInit() {
		cancelAutonomous();
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
	}

	@Override
	public void disabledPeriodic() {
		if (drivetrainFSM != null) {
			drivetrainFSM.update(null);
		}
		updateVisionSafely();
	}

	@Override
	public void testInit() {
		cancelAutonomous();
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
		updateVisionSafely();
	}

	@Override
	public void robotPeriodic() {
		if (drivetrain != null) {
			field.setRobotPose(drivetrain.getPose());
		}
	}

	private void cancelAutonomous() {
		if (autonomousSystem != null) {
			autonomousSystem.cancel();
		} else {
			try {
				resetMechanisms();
			} catch (RuntimeException error) {
				reportInitializationFailure("Mechanism cleanup failed: " + error);
			}
		}
	}

	private void updateMechanisms(Input mechanismInput) {
		if (mechanismFaulted) {
			return;
		}
		try {
			for (FSMSystem<?> mechanism : mechanisms) {
				mechanism.update(mechanismInput);
			}
		} catch (RuntimeException error) {
			mechanismFaulted = true;
			cancelAutonomous();
			reportInitializationFailure(
				"Mechanism update failed; shared mechanisms stopped until robot code restarts: " + error);
		}
	}

	final void resetMechanisms() {
		RuntimeException failure = null;
		for (FSMSystem<?> mechanism : mechanisms) {
			try {
				mechanism.reset();
			} catch (RuntimeException error) {
				mechanismFaulted = true;
				if (failure == null) {
					failure = new IllegalStateException("One or more mechanisms could not reset safely");
				}
				failure.addSuppressed(error);
			}
		}
		if (failure != null) {
			throw failure;
		}
	}

	private void updateVisionSafely() {
		if (visionFSM == null) {
			return;
		}
		try {
			visionFSM.updateVision();
		} catch (RuntimeException error) {
			visionFSM = null;
			reportOptionalVisionFailure(
				"Limelight vision disabled after a runtime failure; drive remains available: " + error);
		}
	}

	@Override
	public void close() {
		try {
			cancelAutonomous();
		} finally {
			try {
				if (drivetrain != null) {
					drivetrain.close();
				}
			} finally {
				super.close();
			}
		}
	}
}
