package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

/**
 * Owns the driver's Xbox controller and exposes chassis actions rather than the raw HID object.
 *
 * <p>The normalized axes use positive forward, positive left, and positive counter-clockwise
 * rotation. The signs account for WPILib's Xbox axis conventions.
 */
public class TeleopInput {
	/** Normalized field-relative demand read from the Xbox sticks. */
	public record DriveAxes(double forward, double left, double counterClockwiseRotation) {
		private static final DriveAxes ZERO = new DriveAxes(0.0, 0.0, 0.0);
	}

	private final XboxController driverController;

	public TeleopInput() {
		driverController = new XboxController(Constants.RobotIntegration.DRIVER_CONTROLLER_PORT);
	}

	/** Whether the Driver Station currently reports the driver controller connected. */
	public boolean isDriverControllerConnected() {
		return driverController.isConnected();
	}

	/** Get normalized drive axes; a disconnected controller always produces zero demand. */
	public DriveAxes getDriveAxes() {
		return mapDriveAxes(
			driverController.isConnected(),
			driverController.getLeftX(),
			driverController.getLeftY(),
			driverController.getRightX());
	}

	/** True once on the Start-button press edge while the controller is connected. */
	public boolean isFieldCentricResetPressed() {
		return driverController.isConnected() && driverController.getStartButtonPressed();
	}

	/**
	 * Convert WPILib Xbox raw axes into normalized forward/left/counter-clockwise demand.
	 * Exposed as a pure function so the controller convention can be unit tested without HID.
	 */
	static DriveAxes mapDriveAxes(boolean connected, double leftX, double leftY, double rightX) {
		if (!connected) {
			return DriveAxes.ZERO;
		}
		return new DriveAxes(-leftY, -leftX, -rightX);
	}
}
