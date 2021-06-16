package frc.robot.systems;

public class FSMSystem {
	public enum FSMState {
		START_STATE,
		OTHER_STATE
	}

	private FSMState currentState;

	/**
	 * Create FSMSystem and initialize to starting state.
	 */
	public FSMSystem() {
		currentState = FSMState.START_STATE;
	}

	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public FSMState getCurrentState() {
		return currentState;
	}

	/**
	 * Update FSM based on new inputs in Teleop mode.
	 */
	public void updateTeleop() {
		switch (currentState) {
			case START_STATE:
				currentState = FSMState.OTHER_STATE;
				break;

			case OTHER_STATE:
				currentState = FSMState.OTHER_STATE;
				break;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}
}
