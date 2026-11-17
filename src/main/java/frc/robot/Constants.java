package frc.robot;

public class Constants {
	public class ElevatorConstants {
		public static final double ELEVATOR_UPPER_THRESHOLD = 0;
		public static final double ELEVATOR_LOWER_THRESHHOLD = 0;
		public static final double MAX_PIVOT_OUT = 0;
		public static final double MAX_PIVOT_IN = 0;
		public static final double PIVOT_MAX_ANGLE = 30.0;
		public static final double PIVOT_MIN_ANGLE = -30.0;

		// Elevator PID Constants
		public static final double ELEVATOR_KS = 0;
		public static final double ELEVATOR_KV = 0;
		public static final double ELEVATOR_KA = 0;
		public static final double ELEVATOR_KP = 0;
		public static final double ELEVATOR_KI = 0;
		public static final double ELEVATOR_KD = 0;
		public static final double ELEVATOR_GEARING = 0;
		public static final double ELEVATOR_CURRENT_LIMIT = 0;
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
		public static final double PIVOT_GEARING = 0;
		public static final double PIVOT_CURRENT_LIMIT = 0;
		public static final double PIVOT_CRUISE_VELO = 0;
		public static final double PIVOT_TARGET_ACCEL = 0;
		public static final double PIVOT_EXPO_KV = 0;

		// Hardware IDs (Change these to match your robot's wiring)
		public static final int LEFT_FRICTION_BRAKE_PWM_CHANNEL = 0;
		public static final int RIGHT_FRICTION_BRAKE_PWM_CHANNEL = 1;

		// Servo positions (0.0 to 1.0) - adjust these to match your physical linkages
		public static final double LEFT_BRAKE_ENGAGED_POS = 0.2;
		public static final double LEFT_BRAKE_RELEASED_POS = 0.8;

		// Note: The right servo may need inverted positions if mounted mirrored to the left
		public static final double RIGHT_BRAKE_ENGAGED_POS = 0.8;
		public static final double RIGHT_BRAKE_RELEASED_POS = 0.2;

		// Tolerance for knowing when the elevator has stopped
		public static final double VELOCITY_STOP_TOLERANCE = 0.05;

		public static final double UPDATE_FREQUENCY = 100;
	}
}
