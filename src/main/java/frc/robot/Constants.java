package frc.robot;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/** Generic template defaults; robot-specific drivetrain hardware belongs in TunerConstants. */
public final class Constants {
	private Constants() { }

	/** Driver controls and optional robot integrations. */
	public static final class RobotIntegration {
		private RobotIntegration() { }

		/** Driver Station USB port for the Xbox controller. */
		public static final int DRIVER_CONTROLLER_PORT = 0;
		/** Driver Station USB port for the mechanism Xbox controller. */
		public static final int MECHANISM_CONTROLLER_PORT = 1;
		/** Per-axis normalized stick deadband. */
		public static final double DRIVER_DEADBAND = 0.08;
		/** Fraction of the Phoenix Tuner X measured free speed to allow (1.0 = full speed). */
		public static final double MAX_LINEAR_SPEED_SCALE = 1.0;

		/** Set true only when this robot has a configured Limelight. */
		public static final boolean ENABLE_LIMELIGHT_VISION = true;
		public static final String LIMELIGHT_NAME = "limelight-four";
		public static final int APRILTAG_PIPELINE = 0;
	}

	/** Initial PathPlanner feedback gains; tune at low speed on the actual robot. */
	public static final class Autonomous {
		private Autonomous() { }

		public static final double TRANSLATION_KP = 5.0;
		public static final double ROTATION_KP = 5.0;
		public static final double TIMEOUT_SECONDS = 15.0;
	}

	/** Generic localization-quality defaults; field bounds are intentionally optional. */
	public static final class VisionDefaults {
		private VisionDefaults() { }

		public static final double MAX_MEASUREMENT_AGE_SECONDS = 0.50;
		public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.05;
		public static final double MAX_TAG_AMBIGUITY = 0.20;
		public static final double MAX_AVERAGE_TAG_DISTANCE_METERS = 6.0;
		// Camera reads can fail transiently while NT4 decodes a published value. Vision is only
		// abandoned after this many consecutive failures (about 0.5 s at 50 Hz).
		public static final int MAX_CONSECUTIVE_READ_FAILURES = 25;
		// The first trustworthy fix replaces the estimator's boot pose. This is the minimum tag
		// count; several tags always qualify, and a lone tag must be no farther than this.
		public static final int MIN_SEED_TAGS = 1;
		// TODO(2473): tune on robot - measure single-tag seed error against distance.
		public static final double MAX_SEED_TAG_DISTANCE_METERS = 3.0;
		// Fused timestamps stay this far behind now so the drivetrain's odometry buffer already
		// holds a sample for the measurement instead of discarding the frame.
		public static final double FUSE_MARGIN_SECONDS = 0.02;
		// Tag-only (MegaTag1) heading is fused with this standard deviation; raise it to trust
		// tag-derived yaw less. Requires the camera's robot-space pose to be configured.
		public static final double HEADING_STD_DEV_RADIANS = 0.20;
		// MegaTag2 uses supplied gyro heading; don't allow vision to override theta.
		public static final Matrix<N3, N1> MEGATAG2_STD_DEVS = VecBuilder.fill(0.70, 0.70, 1.0e6);
	}
}
