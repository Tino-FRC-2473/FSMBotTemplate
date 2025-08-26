package frc.robot.systems;

import frc.robot.TeleopInput;

public interface State<S extends State<S, F>, F extends FSMSystem<S, F>> {
	/**
	 * updates the state.
	 * @param system the fsm, which contains hardware
	 * @param input the teleop input
	 */
	void update(F system, TeleopInput input);

	/**
	 * calculates the next state based on the current state and system hardware.
	 * @param system the fsm, which contains hardware
	 * @param input the teleop input
	 * @return the next state
	 */
	S nextState(F system, TeleopInput input);
}
