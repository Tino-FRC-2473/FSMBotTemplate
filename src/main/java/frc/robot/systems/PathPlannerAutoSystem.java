package frc.robot.systems;

import java.util.Objects;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/** PathPlanner owns drive output only during autonomous; teleop remains manually ticked. */
public final class PathPlannerAutoSystem {
	private static final double MODULE_LOCATION_TOLERANCE_METERS = 0.001;

	private final SendableChooser<Command> chooser;
	private final Runnable stopDrive;
	private final Runnable resetMechanisms;
	private final Timer autoTimer = new Timer();
	private Command activeCommand;
	private boolean runtimeFailed;

	/** Configure and preload GUI autos; callers must isolate failure from teleop initialization. */
	public static PathPlannerAutoSystem create(
		CommandSwerveDrivetrain drivetrain, Runnable resetMechanisms) throws Exception {
		RobotConfig config = RobotConfig.fromGUISettings();
		validateConfig(config, drivetrain.getModuleLocations());
		AutoBuilder.configure(
			drivetrain::getPose,
			drivetrain::resetPose,
			drivetrain::getRobotRelativeSpeeds,
			drivetrain::driveRobotRelative,
			new PPHolonomicDriveController(
				new PIDConstants(Constants.Autonomous.TRANSLATION_KP, 0.0, 0.0),
				new PIDConstants(Constants.Autonomous.ROTATION_KP, 0.0, 0.0)),
			config,
			() -> shouldFlipPaths(DriverStation.getAlliance()),
			drivetrain);
		return new PathPlannerAutoSystem(
			AutoBuilder.buildAutoChooser(), drivetrain::stop, resetMechanisms);
	}

	// Separate command lifecycle from CAN hardware so stops and cancellation can be tested.
	PathPlannerAutoSystem(SendableChooser<Command> chooser, Runnable stopDrive) {
		this(chooser, stopDrive, () -> { });
	}

	/** Bind preloaded commands to drive stop and shared-mechanism reset callbacks. */
	public PathPlannerAutoSystem(
		SendableChooser<Command> chooser, Runnable stopDrive, Runnable resetMechanisms) {
		this.chooser = Objects.requireNonNull(chooser, "chooser");
		this.stopDrive = Objects.requireNonNull(stopDrive, "stopDrive");
		this.resetMechanisms = Objects.requireNonNull(resetMechanisms, "resetMechanisms");
	}

	/** Dashboard selector; the default is PathPlanner's nonmoving "None" option. */
	public SendableChooser<Command> getChooser() {
		return chooser;
	}

	/** Schedule the preloaded selection once at autonomous entry. */
	public void start() {
		cancel();
		if (runtimeFailed) {
			return;
		}
		activeCommand = chooser.getSelected();
		if (activeCommand != null) {
			autoTimer.restart();
			try {
				CommandScheduler.getInstance().schedule(activeCommand);
				if (!activeCommand.isScheduled()) {
					cancel();
				}
			} catch (RuntimeException error) {
				cancel();
				throw error;
			}
		}
	}

	/** Whether shared mechanisms may execute autonomous outputs this loop. */
	public boolean isRunning() {
		return activeCommand != null && !runtimeFailed && activeCommand.isScheduled()
			&& DriverStation.isAutonomousEnabled();
	}

	/** Tick only in autonomous; stop after completion, interruption, or the 15-second deadline. */
	public void update() {
		try {
			if (activeCommand == null) {
				stopDrive.run();
				return;
			}
			if (!DriverStation.isAutonomousEnabled()
				|| !activeCommand.isScheduled()
				|| autoTimer.hasElapsed(Constants.Autonomous.TIMEOUT_SECONDS)) {
				cancel();
				return;
			}
			CommandScheduler.getInstance().run();
			if (!activeCommand.isScheduled()) {
				cancel();
			}
		} catch (RuntimeException error) {
			// An exception inside run() can leave scheduler queues inconsistent. Do not run it again.
			runtimeFailed = true;
			CommandScheduler.getInstance().disable();
			cancel();
			DriverStation.reportError(
				"PathPlanner auto stopped after a runtime failure; restart robot code before retrying: "
					+ error, error.getStackTrace());
		}
	}

	/** Cancel commands, then immediately stop drive and reset even completed one-shot actions. */
	public void cancel() {
		activeCommand = null;
		autoTimer.stop();
		try {
			// This template uses the global command scheduler exclusively for autonomous.
			CommandScheduler.getInstance().cancelAll();
		} catch (RuntimeException error) {
			runtimeFailed = true;
			CommandScheduler.getInstance().disable();
			DriverStation.reportError(
				"Autonomous cancellation failed; restart robot code before retrying: " + error,
				error.getStackTrace());
		} finally {
			// Attempt both cleanup operations even when command end() or drive stop fails.
			try {
				stopDrive.run();
			} catch (RuntimeException error) {
				reportCleanupFailure("drivetrain stop", error);
			} finally {
				try {
					resetMechanisms.run();
				} catch (RuntimeException error) {
					reportCleanupFailure("mechanism reset", error);
				}
			}
		}
	}

	private void reportCleanupFailure(String operation, RuntimeException error) {
		runtimeFailed = true;
		CommandScheduler.getInstance().disable();
		DriverStation.reportError("Autonomous " + operation
			+ " failed; restart robot code before retrying: " + error, error.getStackTrace());
	}

	static boolean shouldFlipPaths(Optional<Alliance> alliance) {
		return alliance.orElse(Alliance.Blue) == Alliance.Red;
	}

	/** Reject incorrect GUI physics or module ordering rather than attempting unsafe following. */
	static void validateConfig(RobotConfig config, Translation2d[] moduleLocations) {
		if (!config.isHolonomic || config.numModules != 4 || moduleLocations.length != 4) {
			throw new IllegalArgumentException(
				"PathPlanner requires four swerve modules in FL, FR, BL, BR order");
		}
		requirePositive(config.massKG, "Robot mass");
		requirePositive(config.MOI, "Robot moment of inertia");
		requirePositive(config.moduleConfig.wheelRadiusMeters, "Wheel radius");
		requirePositive(config.moduleConfig.maxDriveVelocityMPS, "Maximum drive speed");
		requirePositive(config.moduleConfig.wheelCOF, "Wheel coefficient of friction");
		requirePositive(config.moduleConfig.driveCurrentLimit, "Drive current limit");
		for (int index = 0; index < moduleLocations.length; index++) {
			Translation2d expected = moduleLocations[index];
			Translation2d configured = config.moduleLocations[index];
			if (!Double.isFinite(expected.getX()) || !Double.isFinite(expected.getY())
				|| !Double.isFinite(configured.getX()) || !Double.isFinite(configured.getY())
				|| expected.getNorm() <= 0.0 || configured.getNorm() <= 0.0
				|| configured.getDistance(expected) > MODULE_LOCATION_TOLERANCE_METERS) {
				throw new IllegalArgumentException(
					"PathPlanner module " + index + " location/order does not match TunerConstants");
			}
		}
	}

	private static void requirePositive(double value, String name) {
		if (!Double.isFinite(value) || value <= 0.0) {
			throw new IllegalArgumentException(name + " must be finite and positive");
		}
	}
}
