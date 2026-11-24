package frc.robot.systems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

// Third party Hardware Imports
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
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

	private final StatusSignal<Angle> elevatorMotorUp1Pos;
	private final StatusSignal<Angle> elevatorMotorUp2Pos;
	private final StatusSignal<AngularVelocity> elevatorMotorUp1Vel;
	private final StatusSignal<AngularVelocity> elevatorMotorUp2Vel;
	private final StatusSignal<AngularAcceleration> elevatorMotorUp1Accel;
	private final StatusSignal<AngularAcceleration> elevatorMotorUp2Accel;
	private final StatusSignal<Voltage> elevatorMotorUp1Vol;
	private final StatusSignal<Voltage> elevatorMotorUp2Vol;
	private final StatusSignal<Current> elevatorMotorUp1Cur;
	private final StatusSignal<Current> elevatorMotorUp2Cur;

	private final StatusSignal<Angle> elevatorMotorPivot1Pos;
	private final StatusSignal<Angle> elevatorMotorPivot2Pos;
	private final StatusSignal<AngularVelocity> elevatorMotorPivot1Vel;
	private final StatusSignal<AngularVelocity> elevatorMotorPivot2Vel;
	private final StatusSignal<AngularAcceleration> elevatorMotorPivot1Accel;
	private final StatusSignal<AngularAcceleration> elevatorMotorPivot2Accel;
	private final StatusSignal<Voltage> elevatorMotorPivot1Vol;
	private final StatusSignal<Voltage> elevatorMotorPivot2Vol;
	private final StatusSignal<Current> elevatorMotorPivot1Cur;
	private final StatusSignal<Current> elevatorMotorPivot2Cur;

	// Hardware devices should be owned by one and only one system. They must
	// be private to their owner system and may not be used elsewhere.
	private TalonFXWrapper elevatorMotorUp1;
	private TalonFXWrapper elevatorMotorUp2;
	private TalonFXWrapper elevatorMotorPivot1;
	private TalonFXWrapper elevatorMotorPivot2;

	private DigitalInput elevatorUpLimitSwitch;
	private DigitalInput elevatorDownLimitSwitch;
	private DigitalInput elevatorRightLimitSwitch;
	private DigitalInput elevatorLeftLimitSwitch;

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
		Units.inchesToMeters(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD),
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
		elevatorMotorUp1 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_1);
		elevatorMotorUp2 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_2);
		elevatorMotorPivot1 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_1);
		elevatorMotorPivot2 = new TalonFXWrapper(HardwareMap.CAN_ID_ELEVATOR_UP_2);

		elevatorMotorUp1.setPosition(ElevatorConstants.ELEVATOR_LOWER_THRESHHOLD);
		elevatorMotorUp2.setPosition(ElevatorConstants.ELEVATOR_UPPER_THRESHOLD);
		elevatorMotorPivot1.setPosition(0);
		elevatorMotorPivot2.setPosition(0);

		// Get motor status signals
		elevatorMotorUp1Pos = elevatorMotorUp1.getPosition();
		elevatorMotorUp1Vel = elevatorMotorUp1.getVelocity();
		elevatorMotorUp1Accel = elevatorMotorUp1.getAcceleration();
		elevatorMotorUp1Cur = elevatorMotorUp1.getStatorCurrent();
		elevatorMotorUp1Vol = elevatorMotorUp1.getMotorVoltage();

		elevatorMotorUp2Pos = elevatorMotorUp2.getPosition();
		elevatorMotorUp2Vel = elevatorMotorUp2.getVelocity();
		elevatorMotorUp2Accel = elevatorMotorUp2.getAcceleration();
		elevatorMotorUp2Cur = elevatorMotorUp2.getStatorCurrent();
		elevatorMotorUp2Vol = elevatorMotorUp2.getMotorVoltage();

		elevatorMotorPivot1Pos = elevatorMotorPivot1.getPosition();
		elevatorMotorPivot1Vel = elevatorMotorPivot1.getVelocity();
		elevatorMotorPivot1Accel = elevatorMotorPivot1.getAcceleration();
		elevatorMotorPivot1Cur = elevatorMotorPivot1.getStatorCurrent();
		elevatorMotorPivot1Vol = elevatorMotorPivot1.getMotorVoltage();

		elevatorMotorPivot2Pos = elevatorMotorPivot2.getPosition();
		elevatorMotorPivot2Vel = elevatorMotorPivot2.getVelocity();
		elevatorMotorPivot2Accel = elevatorMotorPivot2.getAcceleration();
		elevatorMotorPivot2Cur = elevatorMotorPivot2.getStatorCurrent();
		elevatorMotorPivot2Vol = elevatorMotorPivot2.getMotorVoltage();

		// Initialize TalonFX configurations
		var elevatorUpConfigs = new TalonFXConfiguration();
		var elevatorPivotConfigs = new TalonFXConfiguration();

		// Configure follower motors
		elevatorMotorUp2.setControl(new Follower(elevatorMotorUp1.getDeviceID(),
			MotorAlignmentValue.Opposed));

		elevatorMotorPivot2.setControl(new Follower(elevatorMotorPivot1.getDeviceID(),
			MotorAlignmentValue.Opposed));

		// Configure elevator up motors
		var elevatorUpFeedback = elevatorUpConfigs.Feedback;
		elevatorUpFeedback.SensorToMechanismRatio = ElevatorConstants.ELEVATOR_GEARING;

		var elevatorUpLimits = elevatorUpConfigs.CurrentLimits;
		elevatorUpLimits.StatorCurrentLimit = ElevatorConstants.ELEVATOR_CURRENT_LIMIT;
		elevatorUpLimits.StatorCurrentLimitEnable = true;

		var elevatorUpOutput = elevatorUpConfigs.MotorOutput;
		elevatorUpOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		var elevatorUpSlot0 = elevatorUpConfigs.Slot0;
		elevatorUpSlot0.kS = ElevatorConstants.ELEVATOR_KS;
		elevatorUpSlot0.kV = ElevatorConstants.ELEVATOR_KV;
		elevatorUpSlot0.kA = ElevatorConstants.ELEVATOR_KA;
		elevatorUpSlot0.kP = ElevatorConstants.ELEVATOR_KP;
		elevatorUpSlot0.kI = ElevatorConstants.ELEVATOR_KI;
		elevatorUpSlot0.kD = ElevatorConstants.ELEVATOR_KD;

		var elevatorUpMotionMagic = elevatorUpConfigs.MotionMagic;
		elevatorUpMotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.ELEVATOR_CRUISE_VELO;
		elevatorUpMotionMagic.MotionMagicAcceleration = ElevatorConstants.ELEVATOR_TARGET_ACCEL;
		elevatorUpMotionMagic.MotionMagicExpo_kV = ElevatorConstants.ELEVATOR_EXPO_KV;

		// Configure elevator pivot motors
		var elevatorPivotFeedback = elevatorPivotConfigs.Feedback;
		elevatorPivotFeedback.SensorToMechanismRatio = ElevatorConstants.PIVOT_GEARING;

		var elevatorPivotLimits = elevatorPivotConfigs.CurrentLimits;
		elevatorPivotLimits.StatorCurrentLimit = ElevatorConstants.PIVOT_CURRENT_LIMIT;
		elevatorPivotLimits.StatorCurrentLimitEnable = true;

		var elevatorPivotOutput = elevatorPivotConfigs.MotorOutput;
		elevatorPivotOutput.Inverted = InvertedValue.CounterClockwise_Positive;

		var elevatorPivotSlot0 = elevatorPivotConfigs.Slot0;
		elevatorPivotSlot0.kS = ElevatorConstants.PIVOT_KS;
		elevatorPivotSlot0.kV = ElevatorConstants.PIVOT_KV;
		elevatorPivotSlot0.kA = ElevatorConstants.PIVOT_KA;
		elevatorPivotSlot0.kP = ElevatorConstants.PIVOT_KP;
		elevatorPivotSlot0.kI = ElevatorConstants.PIVOT_KI;
		elevatorPivotSlot0.kD = ElevatorConstants.PIVOT_KD;

		var elevatorPivotMotionMagic = elevatorPivotConfigs.MotionMagic;
		elevatorPivotMotionMagic.MotionMagicCruiseVelocity = ElevatorConstants.PIVOT_CRUISE_VELO;
		elevatorPivotMotionMagic.MotionMagicAcceleration = ElevatorConstants.PIVOT_TARGET_ACCEL;
		elevatorPivotMotionMagic.MotionMagicExpo_kV = ElevatorConstants.PIVOT_EXPO_KV;

		// Apply elevator configurations
		elevatorMotorUp1.getConfigurator().apply(elevatorUpConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
			ElevatorConstants.UPDATE_FREQUENCY,
			elevatorMotorUp1Pos,
			elevatorMotorUp1Vel,
			elevatorMotorUp1Accel,
			elevatorMotorUp1Cur,
			elevatorMotorUp1Vol
		);
		elevatorMotorUp1.optimizeBusUtilization();

		elevatorMotorUp2.getConfigurator().apply(elevatorUpConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
			ElevatorConstants.UPDATE_FREQUENCY,
			elevatorMotorUp2Pos,
			elevatorMotorUp2Vel,
			elevatorMotorUp2Accel,
			elevatorMotorUp2Cur,
			elevatorMotorUp2Vol
		);
		elevatorMotorUp2.optimizeBusUtilization();

		elevatorMotorPivot1.getConfigurator().apply(elevatorPivotConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
			ElevatorConstants.UPDATE_FREQUENCY,
			elevatorMotorPivot1Pos,
			elevatorMotorPivot1Vel,
			elevatorMotorPivot1Accel,
			elevatorMotorPivot1Cur,
			elevatorMotorPivot1Vol
		);
		elevatorMotorPivot1.optimizeBusUtilization();

		elevatorMotorPivot2.getConfigurator().apply(elevatorPivotConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
			ElevatorConstants.UPDATE_FREQUENCY,
			elevatorMotorPivot2Pos,
			elevatorMotorPivot2Vel,
			elevatorMotorPivot2Accel,
			elevatorMotorPivot2Cur,
			elevatorMotorPivot2Vol
		);
		elevatorMotorPivot2.optimizeBusUtilization();

		// Initialize limit switches
		elevatorUpLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_UP_LIMIT_SWITCH);
		elevatorDownLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_DOWN_LIMIT_SWITCH);
		elevatorRightLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_RIGHT_LIMIT_SWITCH);
		elevatorLeftLimitSwitch = new DigitalInput(HardwareMap.ELEVATOR_LEFT_LIMIT_SWITCH);

		// Reset state machine
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
		currentState = ElevatorFSMState.HOLD;

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

		switch (currentState) {
			case HOLD -> handleManualState(input);
			case PIVOT -> handleGroundState(input);
			case ELEVATE -> handleL2State(input);
			default -> throw new IllegalStateException("Invalid state: " + currentState.toString());
		}

		currentState = nextState(input);
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
			return ElevatorFSMState.HOLD;
		}
		switch (currentState) {
			case HOLD:
				if (isPivoting(input)) {
					return ElevatorFSMState.PIVOT;
				} else if (isElevating(input)) {
					return ElevatorFSMState.ELEVATE;
				}
				return ElevatorFSMState.HOLD;

			case ELEVATE:
				if (isElevating(input)) {
					return ElevatorFSMState.ELEVATE;
				} else if (isPivoting(input)) {
					return ElevatorFSMState.PIVOT;
				}
				return ElevatorFSMState.HOLD;

			case PIVOT:
				if (isElevating(input)) {
					return ElevatorFSMState.ELEVATE;
				} else if (isPivoting(input)) {
					return ElevatorFSMState.PIVOT;
				}
				return ElevatorFSMState.HOLD;

			default:
				throw new IllegalStateException("Invalid state: " + currentState.toString());
		}
	}

	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		return elevatorDownLimitSwitch.get();
	}

	/**
	 * Getter for the result of the elevator's upper limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isUpperLimitReached() {
		return elevatorUpLimitSwitch.get();
	}

	/**
	 * Getter for the result of the pivot's left limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isLeftLimitSwitchReached() {
		return elevatorLeftLimitSwitch.get();
	}

	/**
	 * Getter for the result of the pivot's left limit switch.
	 * @return whether the limit is reached
	 */
	private boolean isRightLimitSwitchReached() {
		return elevatorRightLimitSwitch.get();
	}

	private boolean isPivoting(TeleopInput input) {
		if (Math.abs(input.getRightJoystickX()) > ElevatorConstants.JOYSTICK_THRESHOLD
				|| Math.abs(input.getRightJoystickY()) > ElevatorConstants.JOYSTICK_THRESHOLD) {
			return true;
		}
		return false;
	}

	private boolean isElevating(TeleopInput input) {
		if (Math.abs(input.getLeftJoystickX()) > ElevatorConstants.JOYSTICK_THRESHOLD
				|| Math.abs(input.getLeftJoystickY()) > ElevatorConstants.JOYSTICK_THRESHOLD) {
			return true;
		}
		return false;
	}

	private Distance getElevatorpos() {
		return Inches.of(elevatorMotorUp1.getPosition().getValueAsDouble());
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
			elevatorMotorUp1.setPosition(0);
			if (signalInput < 0) {
				elevatorMotorUp1.set(0); //don't go even further down if you hit the lower limit!
				return;
			}
		}

		if (signalInput == 0 && elevatorMotorUp1.getPosition().getValueAsDouble()
			> ElevatorConstants.KG_CHECK.in(Inches)) {
			elevatorMotorUp1.setControl(new VoltageOut(ElevatorConstants.ELEVATOR_KG));
		} else {
			elevatorMotorUp1.set(signalInput * ElevatorConstants.ELEVATOR_MANUAL_SCALE);
		}
	}

	/**
	 * Handle behavior in GROUND.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleGroundState(TeleopInput input) {
		if (isBottomLimitReached()) {
			elevatorMotorUp1.setPosition(0);
		} else {
			elevatorMotorUp1.setControl(
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
		elevatorMotorUp1.setControl(
			motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_L2.in(Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL3.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL3State(TeleopInput input) {
		elevatorMotorUp1.setControl(
				motionRequest.withPosition(ElevatorConstants.ELEVATOR_TARGET_L3.in(Inches))
		);
	}

	/**
	 * Handle behavior in LEVEL4.
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *        the robot is in autonomous mode.
	 */
	private void handleL4State(TeleopInput input) {
		elevatorMotorUp1.setControl(
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
			elevatorMotorUp1.setControl(
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
