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
			};
			sampleAxis(axis, value);
		}
		for (ButtonInput button : ButtonInput.values()) {
			boolean connected = switch (button.controller()) {
				case DRIVE -> driverConnected;
				case MECH -> mechanismConnected;
			};
			sampleButton(button, connected && readButton(button));
		}
	}

	private boolean readButton(ButtonInput button) {
		return switch (button) {
			case DRIVETRAIN_RESEED -> driver.getStartButton();
		};
	}
}
