package frc.robot.systems;

import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

enum IdentityStateSpace implements State<IdentityStateSpace, UselessFSMSystem> {
    STATE {
        @Override
        public void update(UselessFSMSystem fsm, TeleopInput input) { }

        @Override
        public IdentityStateSpace nextState(UselessFSMSystem fsm, TeleopInput input) {
            return STATE;
        }
    };
}

/**
 * this is intended to be used in place of an FSM when the hardware is not present
 */
public class UselessFSMSystem extends FSMSystem<IdentityStateSpace, UselessFSMSystem> {

    @Override
    public void reset() {
        currentState = IdentityStateSpace.STATE;
    }

    @Override
    public void update(TeleopInput input) { }

    @Override
    public boolean updateAutonomous(AutoFSMState autoState) {
        return false;
    }

    @Override
    protected IdentityStateSpace nextState(TeleopInput input) {
        return IdentityStateSpace.STATE;
    }

    @Override
    protected UselessFSMSystem self() {
        return this;
    }
    
}
