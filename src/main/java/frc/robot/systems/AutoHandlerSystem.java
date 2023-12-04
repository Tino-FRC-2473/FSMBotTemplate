package frc.robot.systems;

public class AutoHandlerSystem {
	/* ======================== Constants ======================== */
	// Auto FSM state definitions
	public enum AutoFSMState {
		STATE1,
		STATE2,
		STATE3
	}

	/* ======================== Private variables ======================== */
	//Contains the sequential list of states in the current auto path that must be executed
	private AutoFSMState[] currentStateList;

	//The index in the currentStateList where the currentState is at
	private int currentStateIndex;

	//Stores if the current state has finished executing or not
	private boolean isCurrentStateFinished;

	//FSM Systems that the autoHandlerFSM uses
	private FSMSystem subsystem1;

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state.
	 * Initializes any subsystems such as driveFSM, armFSM, ect.
	 * @param fsm1 the subsystem that the auto handler will call functions on
	 */
	public AutoHandlerSystem(FSMSystem fsm1) {
		subsystem1 = fsm1;
	}

	/* ======================== Public methods ======================== */
	/**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
	public AutoFSMState getCurrentState() {
		return currentStateList[currentStateIndex];
	}

	/**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 * @param path the auto path to be executed
	 */
	public void reset(AutoFSMState[] path) {
		currentStateIndex = 0;
		isCurrentStateFinished = false;
		currentStateList = path;
	}

	/**
	 * This function runs the auto's current state.
	 */
	//&& has boolean short circuiting
	//& doesn't have boolean short circuiting
	public void update() {
		if (currentStateIndex >= currentStateList.length) {
			return;
		}
		System.out.println("In State: " + getCurrentState());
		switch (getCurrentState()) {
			case STATE1:
				isCurrentStateFinished = subsystem1.updateAutonomous(AutoFSMState.STATE1);
				break;
			case STATE2:
				isCurrentStateFinished = subsystem1.updateAutonomous(AutoFSMState.STATE2);
				break;
			case STATE3:
				isCurrentStateFinished = subsystem1.updateAutonomous(AutoFSMState.STATE3);
				break;
			default:
				throw new IllegalStateException("Invalid state: " + getCurrentState().toString());
		}
		nextState();
	}

	/* ======================== Private methods ======================== */
	/**
	 * Checks if the current auto state is finished and if so, transitions to the
	 * next state to be executed.
	 */
	private void nextState() {
		if (isCurrentStateFinished) {
			currentStateIndex++;
			isCurrentStateFinished = false;
		}
	}
}
