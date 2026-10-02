package frc.robot.systems;

import java.util.Objects;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.input.Input;

/**
 * Manually updated state machine shared by teleop and autonomous.
 * Implementing Subsystem supplies command requirements, not an automatic update loop.
 * @param <S> the mechanism's state type
 */
public abstract class FSMSystem<S> implements Subsystem {
	private S currentState;

	/** Return the current state. */
	public S getCurrentState() {
		return currentState;
	}

	/**
	 * Directly replace the state without guards or goal translation. Normal transitions remain
	 * enabled; outputs change on the next regular update, not inside this setter.
	 * @param state state to enter, including reentering the current state
	 */
	public void setState(S state) {
		currentState = Objects.requireNonNull(state, "state");
	}

	/** Internal-transition convenience using the same public setter. */
	public void setCurrentState(S newState) {
		setState(newState);
	}

	/**
	 * Create a command that sets state once on initialization and finishes immediately.
	 * It requires this FSM, but does not tick it, hold its state, or wait for physical completion.
	 * @param state state to assign when the command runs
	 * @return a reusable one-shot command suitable for PathPlanner NamedCommands
	 */
	public Command setStateCommand(S state) {
		Objects.requireNonNull(state, "state");
		return Commands.runOnce(() -> setState(state), this);
	}

	/**
	 * Immediately restore safe state AND safe outputs, even when no update follows.
	 * Called before autonomous and on completion, cancellation, disable, and mode changes.
	 */
	public abstract void reset();

	/**
	 * Apply state outputs and advance normal transitions once per enabled loop.
	 * Null input means no controller input, not "stop": sensor/timer transitions still run.
	 * @param input teleop controller input, or null during autonomous
	 */
	public abstract void update(Input input);

	/**
	 * Read-only transition decision. Controller-driven changes must check for non-null input;
	 * sensor/timer-driven changes can operate in either mode.
	 * @param input teleop controller input, or null during autonomous
	 * @return state for the next update
	 */
	protected abstract S nextState(Input input);
}
