package frc.robot;

public class Constants {
    public static final double UPPER_THRESHOLD = 0;
    public static final double LOWER_THRESHHOLD = 0;
    public static final double MAX_PIVOT_OUT = 0;
    public static final double MAX_PIVOT_IN = 0;
    public static final double ELEVATOR_GEAR_RATIO = 25.0;
    public static final double PIVOT_GEAR_RATIO = 0;
    public static final int CAN_ID_ELEVATOR_LEFT = 0;
    public static final int CAN_ID_ELEVATOR_RIGHT = 0;
    public static final int CAN_ID_PIVOT_LEFT = 0;
    public static final int CAN_ID_PIVOT_RIGHT = 0;
    public static final int ELEVATOR_TOP_LIMIT_SWITCH = 0;
    public static final int ELEVATOR_BOTTOM_LIMIT_SWITCH = 0;
    public static final double PIVOT_MAX_ANGLE = 30.0;
    public static final double PIVOT_MIN_ANGLE = 0;
    public static final double ELEVATOR_MAX_SPEED = 0;
    public static final double PIVOT_MAX_SPEED = 0;
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
}