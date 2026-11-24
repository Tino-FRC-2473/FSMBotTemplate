package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int LEFT_JOYSTICK_PORT = 0;
	private static final int RIGHT_JOYSTICK_PORT = 1;
	private static final int CONTROLLER_PORT = 2;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick leftJoystick;
	private Joystick rightJoystick;
	private XboxController controller;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		leftJoystick = new Joystick(LEFT_JOYSTICK_PORT);

		rightJoystick = new Joystick(RIGHT_JOYSTICK_PORT);

		controller = new XboxController(CONTROLLER_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Left Joystick ------------------------ */
	/**
	 * Get X axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickX() {
		return leftJoystick.getX();
	}
	/**
	 * Get Y axis of Left Joystick.
	 * @return Axis value
	 */
	public double getLeftJoystickY() {
		return leftJoystick.getY();
	}
	/**
	 * Get the value of the shooter button.
	 * @return True if button is pressed
	 */
	public boolean isShooterButtonPressed() {
		return leftJoystick.getRawButton(1);
	}
	/**
	 * Get the value of the intake button.
	 * @return True if button is pressed
	 */
	public boolean isIntakeButtonPressed() {
		return leftJoystick.getRawButton(2);
	}

	/* ------------------------ Right Joystick ------------------------ */
	/**
	 * Get X axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickX() {
		return rightJoystick.getX();
	}
	/**
	 * Get Y axis of Right Joystick.
	 * @return Axis value
	 */
	public double getRightJoystickY() {
		return rightJoystick.getY();
	}

	/**
	 * Get A button in controller .
	 * Used to lower the elevator.
	 * @return whether button is pressed
	 */
	public boolean getElevateDownButton() {
		return controller.getAButton();
	}

	/**
	 * Get Y button in controller .
	 * Used to elevate the elevator.
	 * @return whether button is pressed
	 */
	public boolean getElevateUpButton() {
		return controller.getYButton();
	}

	/**
	 * Get X button in controller .
	 * Used to pivot the elevator right.
	 * @return whether button is pressed
	 */
	public boolean getPivotLeftButton() {
		return controller.getXButton();
	}

	/**
	 * Get B button in controller .
	 * Used to pivot the elevator left.
	 * @return whether button is pressed
	 */
	public boolean getPivotRightButton() {
		return controller.getBButton();
	}

	/* ======================== Private methods ======================== */

}
