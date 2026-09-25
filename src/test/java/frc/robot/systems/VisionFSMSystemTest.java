package frc.robot.systems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import java.util.function.UnaryOperator;
import frc.robot.Constants.VisionDefaults;
import frc.robot.imported.LimelightHelpers.PoseEstimate;
import frc.robot.imported.LimelightHelpers.RawFiducial;
import frc.robot.systems.VisionFSMSystem.Config;
import frc.robot.systems.VisionFSMSystem.FieldBounds;
import frc.robot.systems.VisionFSMSystem.RejectionReason;

class VisionFSMSystemTest {
	private static final double NOW_SECONDS = 10.0;
	private static final Config GENERIC_CONFIG = Config.defaults("limelight-example", 0);

	@Test
	void acceptsFreshFinitePoseWhenNoFieldLayoutIsConfigured() {
		PoseEstimate estimate = estimate(new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(20)), 9.9, 0.05);

		assertEquals(RejectionReason.NONE,
			VisionFSMSystem.validateEstimate(estimate, NOW_SECONDS, GENERIC_CONFIG));
	}

	@Test
	void rejectsEstimateWithoutTags() {
		assertEquals(RejectionReason.NO_TAGS,
			VisionFSMSystem.validateEstimate(new PoseEstimate(), NOW_SECONDS, GENERIC_CONFIG));
	}

	@Test
	void rejectsStaleEstimate() {
		PoseEstimate estimate = estimate(new Pose2d(2.0, 3.0, new Rotation2d()), 9.0, 0.05);

		assertEquals(RejectionReason.STALE,
			VisionFSMSystem.validateEstimate(estimate, NOW_SECONDS, GENERIC_CONFIG));
	}

	@Test
	void fieldBoundsAreOptionalAndConfigurable() {
		PoseEstimate estimate = estimate(new Pose2d(30.0, 3.0, new Rotation2d()), 9.9, 0.05);
		Config boundedConfig = new Config(
			"another-limelight",
			2,
			false,
			new Transform3d(),
			0.5,
			0.05,
			0.2,
			6.0,
			new FieldBounds(0.0, 16.0, 0.0, 8.0),
			VisionDefaults.MEGATAG2_STD_DEVS,
			UnaryOperator.identity());

		assertEquals(RejectionReason.NONE,
			VisionFSMSystem.validateEstimate(estimate, NOW_SECONDS, GENERIC_CONFIG));
		assertEquals(RejectionReason.OUT_OF_FIELD,
			VisionFSMSystem.validateEstimate(estimate, NOW_SECONDS, boundedConfig));
	}

	@Test
	void rejectsAmbiguousEstimate() {
		PoseEstimate estimate = estimate(new Pose2d(2.0, 3.0, new Rotation2d()), 9.9, 0.9);

		assertEquals(RejectionReason.HIGH_AMBIGUITY,
			VisionFSMSystem.validateEstimate(estimate, NOW_SECONDS, GENERIC_CONFIG));
	}

	@Test
	void onlyProcessesEachFiniteTimestampOnce() {
		assertTrue(VisionFSMSystem.isTimestampNew(9.9, Double.NEGATIVE_INFINITY));
		assertFalse(VisionFSMSystem.isTimestampNew(9.9, 9.9));
		assertFalse(VisionFSMSystem.isTimestampNew(9.8, 9.9));
		assertFalse(VisionFSMSystem.isTimestampNew(Double.NaN, 9.9));
	}

	private static PoseEstimate estimate(Pose2d pose, double timestamp, double ambiguity) {
		return new PoseEstimate(
			pose,
			timestamp,
			30.0,
			1,
			0.0,
			2.0,
			1.0,
			new RawFiducial[] {new RawFiducial(1, 0.0, 0.0, 1.0, 2.0, 2.0, ambiguity)},
			true);
	}
}
