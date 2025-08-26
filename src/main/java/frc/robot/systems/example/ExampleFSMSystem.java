package frc.robot.systems.example;

// WPILib Imports

// Third party Hardware Imports
import com.revrobotics.spark.SparkMax;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.motors.SparkMaxWrapper;
import frc.robot.HardwareMap;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.State;

enum FSMStateSpace implements State<FSMStateSpace, ExampleFSMSystem> {

	START_STATE {
		@Override
		public void update(ExampleFSMSystem system, TeleopInput input) {
			system.getExampleMotor().set(0);
		}

		@Override
		public FSMStateSpace nextState(ExampleFSMSystem system, TeleopInput input) {
			return START_STATE;
		}
	},
	OTHER_STATE {
		@Override
		public void update(ExampleFSMSystem system, TeleopInput input) {
			system.getExampleMotor().set(ExampleFSMSystem.MOTOR_RUN_POWER);
		}

		@Override
		public FSMStateSpace nextState(ExampleFSMSystem system, TeleopInput input) {
			return START_STATE;
		}
	};

}

public class ExampleFSMSystem extends FSMSystem<FSMStateSpace, ExampleFSMSystem> {
	/* ======================== Constants ======================== */

	static final float MOTOR_RUN_POWER = 0.1f;

	/* ======================== Private variables ======================== */

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private SparkMax exampleMotor;

	SparkMax getExampleMotor() {
		return exampleMotor;
	}

	/* ======================== Constructor ======================== */
	/**
	 * Create FSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot boots.
	 */
	public ExampleFSMSystem() {
		// Perform hardware init using a wrapper class
		// this is so we can see motor outputs during simulatiuons
		exampleMotor = new SparkMaxWrapper(HardwareMap.CAN_ID_SPARK_SHOOTER,
										SparkMax.MotorType.kBrushless);

		// Reset state system
		reset();
	}

	/* ======================== Public methods ======================== */

	@Override
	public void reset() {
		setCurrentState(FSMStateSpace.START_STATE);
		update(null);
	}

}
