package frc.robot.systems;

import frc.robot.TeleopInput;

public interface State<S extends State<S, F>, F extends FSMSystem<S, F>> {
	void update(F fsm, TeleopInput input);
	S nextState(F fsm, TeleopInput input);
}
