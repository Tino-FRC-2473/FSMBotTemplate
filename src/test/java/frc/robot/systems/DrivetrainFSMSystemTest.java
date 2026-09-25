package frc.robot.systems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

class DrivetrainFSMSystemTest {
	@Test
	void removesValuesInsidePerAxisDeadband() {
		assertEquals(0.0, DrivetrainFSMSystem.applyDeadbandAndClamp(0.05, 0.08), 1.0e-9);
		assertEquals(0.0, DrivetrainFSMSystem.applyDeadbandAndClamp(-0.08, 0.08), 1.0e-9);
	}

	@Test
	void rescalesValuesOutsideDeadbandLinearly() {
		assertEquals(1.0, DrivetrainFSMSystem.applyDeadbandAndClamp(1.0, 0.08), 1.0e-9);
		assertEquals(-1.0, DrivetrainFSMSystem.applyDeadbandAndClamp(-1.0, 0.08), 1.0e-9);
		assertEquals(0.5, DrivetrainFSMSystem.applyDeadbandAndClamp(0.54, 0.08), 1.0e-9);
	}

	@Test
	void clampsOutOfRangeInputsAndIgnoresNonFiniteInput() {
		assertEquals(1.0, DrivetrainFSMSystem.applyDeadbandAndClamp(2.0, 0.08), 1.0e-9);
		assertEquals(-1.0, DrivetrainFSMSystem.applyDeadbandAndClamp(-2.0, 0.08), 1.0e-9);
		assertEquals(0.0, DrivetrainFSMSystem.applyDeadbandAndClamp(Double.NaN, 0.08), 1.0e-9);
	}

	@Test
	void derivesAngularLimitFromConfiguredLinearLimitAndModuleRadius() {
		DrivetrainFSMSystem.Config config = DrivetrainFSMSystem.configFromGeometry(4.8, 0.5, 0.4, 0.08);

		assertEquals(2.4, config.maxLinearSpeedMetersPerSecond(), 1.0e-9);
		assertEquals(6.0, config.maxAngularSpeedRadiansPerSecond(), 1.0e-9);
		assertEquals(0.08, config.deadband(), 1.0e-9);
	}

	@Test
	void fullSpeedScaleUsesGeneratedSpeedWithoutAdditionalReduction() {
		DrivetrainFSMSystem.Config config = DrivetrainFSMSystem.configFromGeometry(4.8, 1.0, 0.4, 0.08);

		assertEquals(4.8, config.maxLinearSpeedMetersPerSecond(), 1.0e-9);
		assertEquals(12.0, config.maxAngularSpeedRadiansPerSecond(), 1.0e-9);
	}

	@Test
	void rejectsInvalidGeneratedSpeedScaleAndModuleGeometry() {
		assertThrows(IllegalArgumentException.class,
			() -> DrivetrainFSMSystem.configFromGeometry(0.0, 1.0, 0.4, 0.08));
		assertThrows(IllegalArgumentException.class,
			() -> DrivetrainFSMSystem.configFromGeometry(4.8, 0.0, 0.4, 0.08));
		assertThrows(IllegalArgumentException.class,
			() -> DrivetrainFSMSystem.configFromGeometry(4.8, 1.1, 0.4, 0.08));
		assertThrows(IllegalArgumentException.class,
			() -> DrivetrainFSMSystem.configFromGeometry(4.8, 1.0, 0.0, 0.08));
	}
}
