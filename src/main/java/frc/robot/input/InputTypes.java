package frc.robot.input;

/** Logical teleop actions; their physical bindings belong only to TeleopInput. */
public final class InputTypes {
	private InputTypes() { }

	public enum AxialInput {
		/** Positive forward, normalized [-1, 1]. */
		DRIVETRAIN_DRIVE_X,
		/** Positive left, normalized [-1, 1]. */
		DRIVETRAIN_DRIVE_Y,
		/** Positive counterclockwise, normalized [-1, 1]. */
		DRIVETRAIN_ROTATE,
		/** Mechanism controller left-X, positive right, normalized [-1, 1]. */
		CLIMBER_MANUAL_CONTROL
	}

	public enum ButtonInput {
		DRIVETRAIN_RESEED,
		DRIVETRAIN_PATHFIND,
		DRIVETRAIN_INVERT_CONTROLS,
		FACE_PASS,
		FACE_HUB,
		BALL_SHAKE_FRONT,
		BALL_SHAKE_SIDE,
		CLIMBER_MANUAL_OVERRIDE,
		CLIMBER_NEXT_STEP,
		CLIMBER_EMERGENCY_ABORT,
		/** Reserved, currently unbound. */
		CLIMBER_AUTO_UP_1,
		/** Reserved, currently unbound. */
		CLIMBER_AUTO_UP_2,
		PASSER_PREP_TOGGLE,
		SHOOTER_PREP_TOGGLE,
		MANUAL_SHOOT_TOGGLE,
		IDLE_SHOOTER_TOGGLE,
		REV_FEEDER,
		INTAKE_BUTTON,
		OUTTAKE_BUTTON,
		FOLD_IN_BUTTON,
		FOLD_OUT_BUTTON,
		PARTIAL_OUT_BUTTON;

		/** Which controller supplies this action; independent of its physical binding. */
		boolean usesDriverController() {
			return switch (this) {
				case DRIVETRAIN_RESEED, DRIVETRAIN_PATHFIND, DRIVETRAIN_INVERT_CONTROLS,
					FACE_PASS, FACE_HUB, BALL_SHAKE_FRONT, BALL_SHAKE_SIDE -> true;
				default -> false;
			};
		}
	}
}
