package frc.robot;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNull;

import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class RobotVisionConfigurationTest {
	@Test
	void limelightVisionDefaultsOff() {
		assertFalse(Constants.RobotIntegration.ENABLE_LIMELIGHT_VISION);
	}

	@Test
	void disabledVisionDoesNotInvokeFactory() {
		AtomicInteger factoryCalls = new AtomicInteger();

		assertNull(Robot.createVisionIfEnabled(false, () -> {
			factoryCalls.incrementAndGet();
			return null;
		}));
		assertEquals(0, factoryCalls.get());
	}
}
