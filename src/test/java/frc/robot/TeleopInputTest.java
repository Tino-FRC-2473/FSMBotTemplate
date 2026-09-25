package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class TeleopInputTest {
	@Test
	void mapsXboxStickSignsToForwardLeftAndCounterClockwise() {
		TeleopInput.DriveAxes axes = TeleopInput.mapDriveAxes(true, -0.4, -0.8, -0.6);

		assertEquals(0.8, axes.forward(), 1.0e-9);
		assertEquals(0.4, axes.left(), 1.0e-9);
		assertEquals(0.6, axes.counterClockwiseRotation(), 1.0e-9);
	}

	@Test
	void disconnectedControllerProducesZeroDemandRegardlessOfAxisValues() {
		TeleopInput.DriveAxes axes = TeleopInput.mapDriveAxes(false, 1.0, -1.0, 0.7);

		assertEquals(0.0, axes.forward(), 1.0e-9);
		assertEquals(0.0, axes.left(), 1.0e-9);
		assertEquals(0.0, axes.counterClockwiseRotation(), 1.0e-9);
	}
}
