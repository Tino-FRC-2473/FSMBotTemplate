package frc.robot.systems;

import frc.robot.input.Input;

/**
 * this is intended to be used in place of an FSM when the hardware is not present.
 */
public class PlaceholderFSMSystem extends FSMSystem<PlaceholderFSMSystem.IdentityStateSpace> {

	/** Single inert state used when a mechanism's hardware is absent. */
	enum IdentityStateSpace {
		/** The only state; it drives no outputs. */
		STATE
	}

	@Override
	public void reset() {
		setCurrentState(IdentityStateSpace.STATE);
	}

	@Override
	public void update(Input input) { }

	@Override
	protected IdentityStateSpace nextState(Input input) {
		return IdentityStateSpace.STATE;
	}

}
