package frc.robot;

// WPILib Imports
import edu.wpi.first.wpilibj.Joystick;

/**
 * Common class for providing driver inputs during Teleop.
 *
 * This class is the sole owner of WPILib input objects and is responsible for
 * polling input values. Systems may query TeleopInput via its getter methods
 * for inputs by value, but may not access the internal input objects.
 */
public class TeleopInput {
	/* ======================== Constants ======================== */
	private static final int JOYSTICK_PORT = 0;

	/* ======================== Private variables ======================== */
	// Input objects
	private Joystick joystick;

	/* ======================== Constructor ======================== */
	/**
	 * Create a TeleopInput and register input devices. Note that while inputs
	 * are registered at robot initialization, valid values will not be provided
	 * by WPILib until teleop mode.
	 */
	public TeleopInput() {
		joystick = new Joystick(JOYSTICK_PORT);
	}

	/* ======================== Public methods ======================== */
	// Getter methods for fetch input values should be defined here.
	// Method names should be descriptive of the behavior, so the
	// control mapping is hidden from other classes.

	/* ------------------------ Joystick ------------------------ */
	/**
	 * Get the value of the on button.
	 * @return True if button is pressed
	 */
	public boolean isOnButtonPressed() {
		return // TODO: Get button value
	}
	/**
	 * Get the value of the off button.
	 * @return True if button is pressed
	 */
	public boolean isOffButtonPressed() {
		return // TODO: Get button value
	}

	/* ======================== Private methods ======================== */

}
