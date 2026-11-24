package frc.robot;

public class Constants {
	public class ElevatorConstants {
		public static final double ELEVATOR_UPPER_THRESHOLD = 50;
		public static final double ELEVATOR_LOWER_THRESHHOLD = 0;
		public static final double PIVOT_MAX_ANGLE = 30.0;
		public static final double PIVOT_MIN_ANGLE = -30.0;

		// Elevator PID Constants
		public static final double ELEVATOR_KS = 0;
		public static final double ELEVATOR_KV = 0;
		public static final double ELEVATOR_KA = 0;
		public static final double ELEVATOR_KP = 0;
		public static final double ELEVATOR_KI = 0;
		public static final double ELEVATOR_KD = 0;
		public static final double ELEVATOR_GEARING = 25.0;
		public static final double ELEVATOR_CURRENT_LIMIT = 60.0;
		public static final double ELEVATOR_CRUISE_VELO = 0;
		public static final double ELEVATOR_TARGET_ACCEL = 0;
		public static final double ELEVATOR_EXPO_KV = 0;

		// Pivot PID Constants
		public static final double PIVOT_KS = 0;
		public static final double PIVOT_KV = 0;
		public static final double PIVOT_KA = 0;
		public static final double PIVOT_KP = 0;
		public static final double PIVOT_KI = 0;
		public static final double PIVOT_KD = 0;
		public static final double PIVOT_GEARING = 1.0;
		public static final double PIVOT_CURRENT_LIMIT = 60.0;
		public static final double PIVOT_CRUISE_VELO = 0;
		public static final double PIVOT_TARGET_ACCEL = 0;
		public static final double PIVOT_EXPO_KV = 0;

		public static final double LEFT_BRAKE_ENGAGED_POS = 0.2;
		public static final double LEFT_BRAKE_RELEASED_POS = 0.8;

		public static final double RIGHT_BRAKE_ENGAGED_POS = 0.8;
		public static final double RIGHT_BRAKE_RELEASED_POS = 0.2;

		public static final double JOYSTICK_DEADBAND = 0.1;
		public static final double ELEVATOR_MANUAL_SCALE = 0.3; // tuning needed
		public static final double PIVOT_MANUAL_SCALE = 0.2; // tuning needed

		public static final double ELEVATOR_MIN_HEIGHT = 0;
		public static final double ELEVATOR_MAX_HEIGHT = ELEVATOR_UPPER_THRESHOLD;

		public static final double ELEVATOR_SPOOL_DIAMETER_INCHES = 1.0;
		public static final double ELEVATOR_CASCADE_MULTIPLIER = 3.0;
		public static final double ELEVATOR_INCHES_PER_ROTATION = Math.PI * ELEVATOR_SPOOL_DIAMETER_INCHES
				* ELEVATOR_CASCADE_MULTIPLIER;

		// Gravity hold
		public static final double ELEVATOR_KG = 0;
		public static final double PIVOT_KG = 0;

		public static final double UPDATE_FREQUENCY = 100;
	}
}
