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
		/** Per-axis normalized stick deadband. */
		public static final double DRIVER_DEADBAND = 0.08;
		/** Fraction of the Phoenix Tuner X measured free speed to allow (1.0 = full speed). */
		public static final double MAX_LINEAR_SPEED_SCALE = 1.0;

		/** Set true only when this robot has a configured Limelight. */
		public static final boolean ENABLE_LIMELIGHT_VISION = false;
		public static final String LIMELIGHT_NAME = "limelight";
		public static final int APRILTAG_PIPELINE = 0;
	}

	/** Generic localization-quality defaults; field bounds are intentionally optional. */
	public static final class VisionDefaults {
		private VisionDefaults() { }

		public static final double MAX_MEASUREMENT_AGE_SECONDS = 0.50;
		public static final double MAX_FUTURE_TIMESTAMP_SECONDS = 0.05;
		public static final double MAX_TAG_AMBIGUITY = 0.20;
		public static final double MAX_AVERAGE_TAG_DISTANCE_METERS = 6.0;
		// MegaTag2 uses supplied gyro heading; don't allow vision to override theta.
		public static final Matrix<N3, N1> MEGATAG2_STD_DEVS = VecBuilder.fill(0.70, 0.70, 1.0e6);
	}
}
