package frc.robot.systems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.motorsims.SimulatedBattery;

// WPILib Imports

// Third party Hardware Imports
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.HardwareMap;
import frc.robot.Robot;

// Robot Imports
import frc.robot.TeleopInput;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.motors.TalonFXWrapper;


public class ElevatorFSMSystem {

	/* ======================== ElevatorConstants ======================== */

	// FSM state definitions
	public enum ElevatorFSMState {
		HOLD,
		ELEVATE,
		PIVOT
	}

	/* ======================== Private variables ======================== */

	private ElevatorFSMState currentState;
	private MotionMagicVoltage motionRequest;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFX elevatorMotor1;
	private TalonFX elevatorMotor2;

	private DigitalInput groundLimitSwitch;

	private final DCMotor gearbox = DCMotor.getKrakenX60(1);

	private final LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
		gearbox,
		Units.lbsToKilograms(13.8),
		Units.inchesToMeters(1),
		15);

	private ElevatorSim elevatorSim = new ElevatorSim(
		elevatorSystem,
		gearbox,
		0,
		Units.inchesToMeters(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD.in(Inches)),
		true,
		0);

	/* ======================== Constructor ======================== */

	/**
	 * Create ElevatorFSMSystem and initialize to starting state. Also perform any
	 * one-time initialization or configuration of hardware required. Note
	 * the constructor is called only once when the robot bootss
	 */
	public ElevatorFSMSystem() {
		// Perform hardware init
		elevatorMotor1 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_1);
		elevatorMotor2 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_2);

		motionRequest = new MotionMagicVoltage(0);

		var talonFXConfigs = new TalonFXConfiguration();

		var outputConfigs = talonFXConfigs.MotorOutput;
		outputConfigs.NeutralMode = NeutralModeValue.Brake;

		// apply sw limit
		var swLimitSwitch = talonFXConfigs.SoftwareLimitSwitch;
		swLimitSwitch.ForwardSoftLimitEnable = true; // enable top limit
		swLimitSwitch.ReverseSoftLimitEnable = true; // enable bottom limit
		swLimitSwitch.ForwardSoftLimitThreshold = ElevatorConstants.ELEVATOR_UPPER_THRESHOLD
			.in(Inches);
		swLimitSwitch.ReverseSoftLimitThreshold = Inches.of(0).in(Inches);

		var sensorConfig = talonFXConfigs.Feedback;
		sensorConfig.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_ROTS_TO_INCHES;

		var slot0Configs = talonFXConfigs.Slot0;
		slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
		slot0Configs.kG = ElevatorConstants.ELEVATOR_KG;
		slot0Configs.kS = ElevatorConstants.ELEVATOR_KS;
		slot0Configs.kV = ElevatorConstants.ELEVATOR_KV;
		slot0Configs.kA = ElevatorConstants.ELEVATOR_KA;
		slot0Configs.kP = ElevatorConstants.ELEVATOR_KP;
		slot0Configs.kI = ElevatorConstants.ELEVATOR_KI;
		slot0Configs.kD = ElevatorConstants.ELEVATOR_KD;
		slot0Configs.StaticFeedforwardSign = StaticFeedforwardSignValue.UseClosedLoopSign;

		var motionMagicConfigs = talonFXConfigs.MotionMagic;
		motionMagicConfigs.MotionMagicCruiseVelocity = ElevatorConstants.ELEVATOR_CRUISE_VELO;
		motionMagicConfigs.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_TARGET_ACCEL;
		motionMagicConfigs.MotionMagicExpo_kV = ElevatorConstants.ELEVATOR_EXPO_KV;

		elevatorMotor1.getConfigurator().apply(talonFXConfigs);

		BaseStatusSignal.setUpdateFrequencyForAll(
				ElevatorConstants.UPDATE_FREQUENCY_HZ,
				elevatorMotor1.getPosition(),
				elevatorMotor1.getVelocity(),
				elevatorMotor1.getAcceleration(),
				elevatorMotor1.getMotorVoltage(),
				elevatorMotor1.getRotorPosition(),
				elevatorMotor1.getRotorVelocity()
		);

		elevatorMotor1.optimizeBusUtilization();
			// MUST set brake after applying other configs

		// Initialize limit switch
		groundLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_GROUND_LIMIT_SWITCH_DIO_PORT);

		// Reset state machine

		elevatorMotor1.setPosition(0);

		reset();
	}

	/* ======================== Public methods ======================== */

	/**
	 * Get the current FSM state.
	 * @return current FSM state
	 */
	public ElevatorFSMState getCurrentState() {
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
	public void reset() {
		currentState = ElevatorFSMState.MANUAL;

		// Call one tick of update to ensure outputs reflect start state
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		if (input == null) {
			return;
		}

		if (Robot.isSimulation()) {
			double partOfWayUp =
				elevatorSim.getPositionMeters()
				/ Units.inchesToMeters(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD.in(Inches));
			// double partOfWayUpPerSecond =
			// 	elevatorSim.getVelocityMetersPerSecond()
			// 	/ Units.inchesToMeters(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD.in(Inches));
			Logger.recordOutput("part way up", partOfWayUp);
			((TalonFXWrapper) elevatorMotor1).setRawPosition(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD
				.in(Inches) * partOfWayUp);
			// elevatorMotor1.getSimState()
			//.setRotorVelocity(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD.in(Inches)
			//* partOfWayUpPerSecond);

		}

		elevatorSim.setInputVoltage(elevatorMotor1.get() * SimConstants.BATTERY_VOLTAGE);
		Logger.recordOutput("elevator setpoint", elevatorMotor1.get());
		Logger.recordOutput("simulated battery", SimulatedBattery.getBatteryVoltage().in(Volts));
		elevatorSim.update(ElevatorConstants.UPDATE_PERIOD_SECS);

		switch (currentState) {
			case MANUAL -> handleManualState(input);
			case GROUND -> handleGroundState(input);
			case LEVEL2 -> handleL2State(input);
			case LEVEL3 -> handleL3State(input);
			case LEVEL4 -> handleL4State(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
	}

	/**
	 * Updates the logging information for the elevator system.
	 */
	public void updateLogging() {
		Logger.recordOutput("Elevator encoder", elevatorMotor1.getPosition().getValueAsDouble());
		Logger.recordOutput("Elevator velocity", elevatorMotor1.getVelocity().getValueAsDouble());

		Logger.recordOutput("Elevator bottom limit switch pressed", isBottomLimitReached());

		Logger.recordOutput("Elevator State", currentState.toString());
		Logger.recordOutput("Elevator Voltage", elevatorMotor1.getMotorVoltage().getValueAsDouble());

		Logger.recordOutput("Elevator Accel", elevatorMotor1.getAcceleration().getValueAsDouble());

		Logger.recordOutput("Elev Inrage GRND?", inRange(getElevatorpos(),
			ElevatorConstants.ELEVATOR_TARGET_GROUND));

		Logger.recordOutput("Elev Inrage L4?", inRange(getElevatorpos(),
			ElevatorConstants.ELEVATOR_TARGET_L4));

		Logger.recordOutput("ROTR POS", elevatorMotor1.getRotorPosition().getValueAsDouble());
		Logger.recordOutput("ROTR VELO", elevatorMotor1.getRotorVelocity().getValueAsDouble());

		// telemetry and logging
		MechLogging.getInstance().updateElevatorPose3d(Angle.ofBaseUnits(
			elevatorSim.getPositionMeters(), Radians
		));
	}

	/* ======================== Private methods ======================== */
	/**
 	 * Decide the next state to transition to. This is a function of the inputs
 	 * and the current state of this FSM. This method should not have any side
 	 * effects on outputs. In other words, this method should only read or get
 	 * values to decide what state to go to.
 	 * @param input Global TeleopInput if robot in teleop mode or null if
 	 *        the robot is in autonomous mode.
 	 * @return FSM state for the next iteration
 	 */
	private ElevatorFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ElevatorFSMState.MANUAL;
		}
		switch (currentState) {
			case MANUAL:
				if (input.isGroundButtonPressed()
					&& !isBottomLimitReached()
					&& !input.isL4ButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.GROUND;
				}
				if (input.isL2ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.LEVEL2;
				}
				if (input.isL3ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isL4ButtonPressed()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()) {
					return ElevatorFSMState.LEVEL3;
				}
				if (input.isL4ButtonPressed()
					&& funnelSystem.isHoldingCoral()
					&& !input.isGroundButtonPressed()
					&& !input.isL2ButtonPressed()
					&& !input.isL3ButtonPressed()) {
					return ElevatorFSMState.LEVEL4;
				}
				return ElevatorFSMState.MANUAL;

			case GROUND:
				if (isBottomLimitReached() || inRange(getElevatorpos(),
					ElevatorConstants.ELEVATOR_TARGET_GROUND)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.GROUND;

			case LEVEL2:
				if (inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L2)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL2;

			case LEVEL3:
				if (inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L3)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL3;

			case LEVEL4:
				if (inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L4)) {
					return ElevatorFSMState.MANUAL;
				}
				return ElevatorFSMState.LEVEL4;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		if (Robot.isSimulation()) {
			return elevatorMotor1.getPosition().getValueAsDouble()
				< ElevatorConstants.ELEVATOR_INRANGE_VALUE.in(Inches);
		}
		return groundLimitSwitch.get(); // switch is normally open
	}

	private boolean inRange(Distance currentPos, Distance targetPos) {
		return (currentPos.compareTo(targetPos.minus(ElevatorConstants.ELEVATOR_INRANGE_VALUE)) > 0)
				&& currentPos.compareTo(targetPos.plus(ElevatorConstants.ELEVATOR_INRANGE_VALUE)) < 0;
	}

	private Distance getElevatorpos() {
		return Inches.of(elevatorMotor1.getPosition().getValueAsDouble());
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * Handle behavior in MANUAL.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleManualState(TeleopInput input) {
		double signalInput = input.getManualElevatorMovementInput();
		signalInput = MathUtil.applyDeadband(signalInput,
			ElevatorConstants.ELEVATOR_JOYSTICK_INPUT_DEADBAND);
		if (isBottomLimitReached()) {
			elevatorMotor1.setPosition(0);
			if (signalInput < 0) {
				elevatorMotor1.set(0); //don't go even further down if you hit the lower limit!
				return;
			}
		}

		if (signalInput == 0 && elevatorMotor1.getPosition().getValueAsDouble()
			> ElevatorConstants.KG_CHECK.in(Inches)) {
			elevatorMotor1.setControl(new VoltageOut(ElevatorConstants.ELEVATOR_KG));
		} else {
			elevatorMotor1.set(signalInput * ElevatorConstants.ELEVATOR_MANUAL_SCALE);
		}
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		if (isBottomLimitReached()) {
			elevatorMotor1.setPosition(0);
		} else {
			elevatorMotor1.setControl(
				motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_GROUND.in(Inches))
			);
		}
	}

	/**
	 * Handle behavior in LEVEL2.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL2State(TeleopInput input) {
		elevatorMotor1.setControl(
			motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_L2.in(Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL3State(TeleopInput input) {
		elevatorMotor1.setControl(
				motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_L3.in(Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL4State(TeleopInput input) {
		elevatorMotor1.setControl(
				motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_L4.in(Inches))
		);
	}

	/**
	 * Is elevator at L2 boolean accessor.
	 * @return whether or not elevator is at L2.
	 */
	public boolean isElevatorAtL2() {
		return inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L2);
	}

	/**
	 * Is elevator at L3 boolean accessor.
	 * @return whether or not elevator is at L3.
	 */
	public boolean isElevatorAtL3() {
		return inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L3);
	}

	/**
	 * Is elevator at L4 boolean accessor.
	 * @return whether or not elevator is at L4.
	 */
	public boolean isElevatorAtL4() {
		return inRange(getElevatorpos(), ElevatorConstants.ELEVATOR_TARGET_L4);
	}

	/**
	 * Is elevator at ground boolean accessor.
	 * @return whether or not elevator is at ground.
	 */
	public boolean isElevatorAtGround() {
		return isBottomLimitReached();
	}

	/* ---- Elevator Commands ---- */

	/** Superclass for elevator commands. */
	abstract class ElevatorCommand extends Command {
		private Distance target;

		@Override
		public void execute() {
			elevatorMotor1.setControl(
				motionRequest.withPosition(target.in(Inches))
			);
		}

		@Override
		public boolean isFinished() {
			return inRange(getElevatorpos(), target);
		}

		@Override
		public void end(boolean interrupted) { }

		protected void setTarget(Distance newTarget) {
			this.target = newTarget;
		}

		protected Distance getTarget() {
			return this.target;
		}
	}

	/** A command that waits for one second. */
	class WaitCommand extends Command {
		private Timer timer;

		@Override
		public void initialize() {
			timer = new Timer();
			timer.reset();
			timer.start();
		}

		@Override
		public boolean isFinished() {
			return timer.get() > 1;
		}

		@Override
		public void end(boolean interrupted) {
			timer.reset();
		}
	}

	/** A command that moves the elevator to the Ground position. */
	class ElevatorGroundCommand extends ElevatorCommand {
		ElevatorGroundCommand() {
			this.setTarget(ElevatorConstants.ELEVATOR_TARGET_GROUND);
		}
	}

	/** A command that moves the elevator to the L2 position. */
	class ElevatorL2Command extends ElevatorCommand {
		ElevatorL2Command() {
			this.setTarget(ElevatorConstants.ELEVATOR_TARGET_L2);
		}
	}

	/** A command that moves the elevator to the L3 position. */
	class ElevatorL3Command extends ElevatorCommand {
		ElevatorL3Command() {
			this.setTarget(ElevatorConstants.ELEVATOR_TARGET_L3);
		}
	}

	/** A command that moves the elevator to the L4 position. */
	class ElevatorL4Command extends ElevatorCommand {
		ElevatorL4Command() {
			this.setTarget(ElevatorConstants.ELEVATOR_TARGET_L4);
		}
	}

	/**
	 * Creates a Command to move the elevator to the ground position.
	 * @return A new elevator ground command.
	 */
	public Command elevatorGroundCommand() {
		return new ElevatorGroundCommand();
	}

	/**
	 * Creates a Command to move the elevator to the L2 position.
	 * @return A new elevator L2 command.
	 */
	public Command elevatorL2Command() {
		return new ElevatorL2Command();
	}

	/**
	 * Creates a Command to move the elevator to the L3 position.
	 * @return A new elevator L3 command.
	 */
	public Command elevatorL3Command() {
		return new ElevatorL3Command();
	}

	/**
	 * Creates a Command to move the elevator to the L4 position.
	 * @return A new elevator L4 command.
	 */
	public Command elevatorL4Command() {
		return new ElevatorL4Command();
	}

	/**
	 * Creates a Command that waits for a specified duration.
	 * @return A new wait command.
	 */
	public Command waitCommand() {
		return new WaitCommand();
	}
}