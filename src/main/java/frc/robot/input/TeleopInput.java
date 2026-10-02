package frc.robot.input;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;

/** Sole owner of Xbox devices; Robot polls once before any teleop FSM reads the snapshot. */
public final class TeleopInput extends Input {
	/** WPILib's usual threshold for a trigger-backed button. */
	public static final double TRIGGER_THRESHOLD = 0.5;

	private final XboxController driver;
	private final XboxController mechanism;

	public TeleopInput() {
		driver = new XboxController(Constants.RobotIntegration.DRIVER_CONTROLLER_PORT);
		mechanism = new XboxController(Constants.RobotIntegration.MECHANISM_CONTROLLER_PORT);
	}

	/** Synchronize at teleop entry; held buttons must release before becoming active. */
	public void synchronize() {
		sample(true);
	}

	/** Refresh exactly once per teleop iteration, before reading any getters. */
	public void poll() {
		sample(false);
	}

	private void sample(boolean synchronize) {
		boolean driverConnected = driver.isConnected();
		boolean mechanismConnected = mechanism.isConnected();
		beginSample(driverConnected, mechanismConnected, synchronize);
		for (AxialInput axis : AxialInput.values()) {
			double value = switch (axis) {
				case DRIVETRAIN_DRIVE_X -> driverConnected ? -driver.getLeftY() : 0.0;
				case DRIVETRAIN_DRIVE_Y -> driverConnected ? -driver.getLeftX() : 0.0;
				case DRIVETRAIN_ROTATE -> driverConnected ? -driver.getRightX() : 0.0;
				case CLIMBER_MANUAL_CONTROL -> mechanismConnected ? mechanism.getLeftX() : 0.0;
			};
			sampleAxis(axis, value);
		}
		for (ButtonInput button : ButtonInput.values()) {
			boolean connected = button.usesDriverController() ? driverConnected : mechanismConnected;
			sampleButton(button, connected && readButton(button));
		}
	}

	private boolean readButton(ButtonInput button) {
		return switch (button) {
			case DRIVETRAIN_RESEED -> driver.getStartButton();
			case DRIVETRAIN_PATHFIND -> driver.getYButton();
			case DRIVETRAIN_INVERT_CONTROLS -> driver.getBackButton();
			case FACE_HUB -> driver.getRightTriggerAxis() > TRIGGER_THRESHOLD;
			case FACE_PASS -> driver.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
			case BALL_SHAKE_FRONT -> driver.getXButton();
			case BALL_SHAKE_SIDE -> driver.getAButton();
			case PARTIAL_OUT_BUTTON -> mechanism.getStartButton();
			case INTAKE_BUTTON -> mechanism.getBButton();
			case OUTTAKE_BUTTON -> mechanism.getAButton();
			case FOLD_IN_BUTTON -> mechanism.getYButton();
			case FOLD_OUT_BUTTON -> mechanism.getXButton();
			case CLIMBER_MANUAL_OVERRIDE -> mechanism.getPOV() == 0;
			case CLIMBER_NEXT_STEP -> mechanism.getPOV() == 270;
			case CLIMBER_EMERGENCY_ABORT -> mechanism.getPOV() == 180;
			case PASSER_PREP_TOGGLE -> mechanism.getRightBumperButton();
			case SHOOTER_PREP_TOGGLE -> mechanism.getLeftTriggerAxis() > TRIGGER_THRESHOLD;
			case MANUAL_SHOOT_TOGGLE -> mechanism.getLeftBumperButton();
			case REV_FEEDER -> mechanism.getRightTriggerAxis() > TRIGGER_THRESHOLD;
			case IDLE_SHOOTER_TOGGLE -> mechanism.getBackButton();
			case CLIMBER_AUTO_UP_1, CLIMBER_AUTO_UP_2 -> false;
		};
	}
}
