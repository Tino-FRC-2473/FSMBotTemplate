package frc.robot.systems;

import frc.robot.TeleopInput;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

/**
 * This is a superclass for FSMs with NECCESARY methods to implement
 * 
 * Start implementing an FSM by writing this in a new java file:
 * 
 * enum FSMState {
 *      // add states here
 * }
 * public class _______ extends FSMSystem<FSMState> {
 *      ...
 * }
 * 
 * Your compiler / IDE will tell you what methods you need to implement
 * You should also have state handlers shown in the example
 */
public abstract class FSMSystem<S extends State<S, F>, F extends FSMSystem<S, F>> {
    /** 
     * the current state, defined as part of the provided statespace
     */
    protected S currentState;

    /**
	 * Return current FSM state.
	 * @return Current FSM state
	 */
    public S getCurrentState() {
        return currentState;
    }

    /**
	 * Reset this system to its start state. This may be called from mode init
	 * when the robot is enabled.
	 *
	 * Note this is distinct from the one-time initialization in the constructor
	 * as it may be called multiple times in a boot cycle,
	 * Ex. if the robot is enabled, disabled, then reenabled.
	 */
    public abstract void reset();

    /**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
    public void update(TeleopInput input) {
        currentState.update(self(), input);
        currentState = nextState(input);
    }

    /**
	 * Performs specific action based on the autoState passed in.
     * This should be overridden if the child doesn't use commands for autonomous
	 * @param autoState autoState that the subsystem executes.
	 * @return if the action carried out in this state has finished executing
	 */
    public boolean updateAutonomous(AutoFSMState autoState) {
        return true;
    }

    /**
	 * Decide the next state to transition to. This is a function of the inputs
	 * and the current state of this FSM. This method should not have any side
	 * effects on outputs. In other words, this method should only read or get
	 * values to decide what state to go to.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
    protected S nextState(TeleopInput input) {
        return currentState.nextState(self(), input);
    }
    
    /**
     * Returns this as the child
     * @return this as the child
     */
    @SuppressWarnings("unchecked")
    protected F self() {
        return (F) this;
    }
}
