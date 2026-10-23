package frc.robot.input;

import static frc.robot.input.InputTypes.ControllerType.DRIVE;
import static frc.robot.input.InputTypes.ControllerType.MECH;

/** Logical teleop actions; their physical bindings belong only to TeleopInput. */
public final class InputTypes {
	private InputTypes() { }

	public enum ControllerType {
		DRIVE {
			@Override
			public boolean isDrive() {
				return true;
			}

			@Override
			public boolean isMech() {
				return false;
			}
		},
		MECH {
			@Override
			public boolean isDrive() {
				return false;
			}

			@Override
			public boolean isMech() {
				return true;
			}
		};

		/**
		 * @return Whether this enum constant represents the drive controller.
		 */
		public abstract boolean isDrive();

		/**
		 * @return Whether this enum constant represents the mech controller.
		 */
		public abstract boolean isMech();
	}
	public interface InputType {
		/**
		 * @return Enum constant representing which controller this input is from.
		 */
		ControllerType controller();
	}

	public enum AxialInput implements InputType {
		/** Positive forward, normalized [-1, 1]. */
		DRIVETRAIN_DRIVE_X(DRIVE),
		/** Positive left, normalized [-1, 1]. */
		DRIVETRAIN_DRIVE_Y(DRIVE),
		/** Positive counterclockwise, normalized [-1, 1]. */
		DRIVETRAIN_ROTATE(DRIVE);

		private ControllerType controllerType;
		AxialInput(ControllerType controller) {
			controllerType = controller;
		}

		@Override
		public ControllerType controller() {
			return controllerType;
		}
	}

	public enum ButtonInput implements InputType {
		DRIVETRAIN_RESEED(DRIVE);

		private ControllerType controllerType;
		ButtonInput(ControllerType controller) {
			controllerType = controller;
		}

		@Override
		public ControllerType controller() {
			return controllerType;
		}
	}
}
