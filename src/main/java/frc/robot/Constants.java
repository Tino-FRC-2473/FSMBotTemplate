package frc.robot;

// Unit imports
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public class Constants {
	public class ElevatorConstants {
		public static final Distance ELEVATOR_UPPER_THRESHOLD = Inches.of(50);
		public static final double ELEVATOR_LOWER_THRESHHOLD = 0;
		// public static final double MAX_PIVOT_OUT = 0;
		// public static final double MAX_PIVOT_IN = 0;
		// public static final double PIVOT_MAX_ANGLE = 30.0;
		// public static final double PIVOT_MIN_ANGLE = -30.0;

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

		public static final double UPDATE_PERIOD_SECS = 0.02;

		public static final Distance ELEVATOR_MIN_HEIGHT = Inches.of(0);
		public static final Distance ELEVATOR_MAX_HEIGHT = ELEVATOR_UPPER_THRESHOLD;
		public static final Distance ELEVATOR_SLOW_ZONE = Inches.of(4); // slow down near limits
		public static final double ELEVATOR_SLOW_ZONE_MIN_SCALE = 0.15;

		public static final double ELEVATOR_SPOOL_DIAMETER_INCHES = 1.0;
		public static final double ELEVATOR_CASCADE_MULTIPLIER = 3.0;
		public static final double ELEVATOR_INCHES_PER_ROTATION =
			Math.PI * ELEVATOR_SPOOL_DIAMETER_INCHES * ELEVATOR_CASCADE_MULTIPLIER;

		public static final Angle PIVOT_MAX_FORWARD = Degrees.of(30);
		public static final Angle PIVOT_MAX_BACK = Degrees.of(-30);
		public static final Angle PIVOT_SLOW_ZONE = Degrees.of(5);
		public static final double PIVOT_SLOW_ZONE_MIN_SCALE = 0.15;

		// Gravity hold
		public static final double ELEVATOR_KG = 0;
		public static final double PIVOT_KG = 0;
		public static final Distance KG_CHECK = Inches.of(1);

		public static final double ELEVATOR_MANUAL_SCALE = 0.3;
		public static final double ELEVATOR_JOYSTICK_INPUT_DEADBAND = 0.1;
		public static final double PIVOT_MANUAL_SCALE = 0.25;
		public static final double PIVOT_JOYSTICK_INPUT_DEADBAND = 0.1;

		// Tolerances
		public static final Distance ELEVATOR_INRANGE_VALUE = Inches.of(0.5);
		public static final Angle PIVOT_INRANGE_VALUE = Degrees.of(1);

		public static final Distance ELEVATOR_TARGET_GROUND = Inches.of(0);
		public static final Distance ELEVATOR_TARGET_L2 = Inches.of(10);
		public static final Distance ELEVATOR_TARGET_L3 = Inches.of(20);
		public static final Distance ELEVATOR_TARGET_L4 = Inches.of(30);

		public static final Angle PIVOT_PRESET_FULL_FORWARD = PIVOT_MAX_FORWARD;
		public static final Angle PIVOT_PRESET_FULL_BACK = PIVOT_MAX_BACK;
		public static final Angle PIVOT_PRESET_MID_FORWARD = Degrees.of(15);
		public static final Angle PIVOT_PRESET_MID_BACK = Degrees.of(-15);
		public static final Angle PIVOT_PRESET_VERTICAL = Degrees.of(0);

		public static final Distance ELEVATOR_MOTOR_DISAGREEMENT = Inches.of(1);
		public static final Angle PIVOT_MOTOR_DISAGREEMENT = Degrees.of(3);
		public static final Distance LIMIT_SWITCH_MISMATCH = Inches.of(1);
		public static final double STALL_CURRENT_AMPS = 50.0;
		public static final double STALL_TIME_SECS = 0.5;
		public static final double PRESET_TIMEOUT_SECS = 3.0;

		public static final Angle FAULT_HOME_ANGLE = Degrees.of(0);
		public static final Distance FAULT_HOME_HEIGHT = Inches.of(0);
		public static final double FAULT_HOME_ELEVATOR_SCALE = 0.15;
		public static final double FAULT_HOME_PIVOT_SCALE = 0.15;
	}
}
