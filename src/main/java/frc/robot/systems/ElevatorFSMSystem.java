package frc.robot.systems;

// Third party Hardware Imports
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
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.HardwareMap;

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

	private final VoltageOut gravityRequest = new VoltageOut(0);
	private final MotionMagicVoltage elevatorHoldRequest = new MotionMagicVoltage(0);
	private final MotionMagicVoltage pivotHoldRequest = new MotionMagicVoltage(0);
	private double elevatorHoldTarget; // mechanism rotations
	private double pivotHoldTarget; // mechanism rotations

	private final DCMotor gearbox = DCMotor.getKrakenX60(1);

	private final LinearSystem<N2, N1, N2> elevatorSystem = LinearSystemId.createElevatorSystem(
			gearbox,
			Units.lbsToKilograms(13.8),
			Units.inchesToMeters(1),
			15);

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
				elevatorMotorUp1Vol);
		elevatorMotorUp1.optimizeBusUtilization();

		elevatorMotorUp2.getConfigurator().apply(elevatorUpConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
				ElevatorConstants.UPDATE_FREQUENCY,
				elevatorMotorUp2Pos,
				elevatorMotorUp2Vel,
				elevatorMotorUp2Accel,
				elevatorMotorUp2Cur,
				elevatorMotorUp2Vol);
		elevatorMotorUp2.optimizeBusUtilization();

		elevatorMotorPivot1.getConfigurator().apply(elevatorPivotConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
				ElevatorConstants.UPDATE_FREQUENCY,
				elevatorMotorPivot1Pos,
				elevatorMotorPivot1Vel,
				elevatorMotorPivot1Accel,
				elevatorMotorPivot1Cur,
				elevatorMotorPivot1Vol);
		elevatorMotorPivot1.optimizeBusUtilization();

		elevatorMotorPivot2.getConfigurator().apply(elevatorPivotConfigs);
		BaseStatusSignal.setUpdateFrequencyForAll(
				ElevatorConstants.UPDATE_FREQUENCY,
				elevatorMotorPivot2Pos,
				elevatorMotorPivot2Vel,
				elevatorMotorPivot2Accel,
				elevatorMotorPivot2Cur,
				elevatorMotorPivot2Vol);
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
	 *
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
		elevatorHoldTarget = getElevatorPos();
		pivotHoldTarget = getPivotPos();
		update(null);
	}

	/**
	 * Update FSM based on new inputs. This function only calls the FSM state
	 * specific handlers.
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 */
	public void update(TeleopInput input) {

		if (input == null) {
			handleHoldState(null);
			return;
		}

		switch (currentState) {
			case HOLD -> handleHoldState(input);
			case PIVOT -> handlePivotState(input);
			case ELEVATE -> handleElevateState(input);
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
	 *
	 * @param input Global TeleopInput if robot in teleop mode or null if
	 *              the robot is in autonomous mode.
	 * @return FSM state for the next iteration
	 */
	private ElevatorFSMState nextState(TeleopInput input) {
		if (input == null) {
			return ElevatorFSMState.HOLD;
		}
		// Same priority from every state: elevate > pivot > hold
		if (isElevating(input)) {
			return ElevatorFSMState.ELEVATE;
		}
		if (isPivoting(input)) {
			return ElevatorFSMState.PIVOT;
		}
		return ElevatorFSMState.HOLD;
	}

	/**
	 * Getter for the result of the elevator's bottom limit switch.
	 *
	 * @return whether the limit is reached
	 */
	private boolean isBottomLimitReached() {
		return elevatorDownLimitSwitch.get();
	}

	/**
	 * Getter for the result of the elevator's upper limit switch.
	 *
	 * @return whether the limit is reached
	 */
	private boolean isUpperLimitReached() {
		return elevatorUpLimitSwitch.get();
	}

	/**
	 * Getter for the result of the pivot's left limit switch.
	 *
	 * @return whether the limit is reached
	 */
	private boolean isLeftLimitSwitchReached() {
		return elevatorLeftLimitSwitch.get();
	}

	/**
	 * Getter for the result of the pivot's right limit switch.
	 *
	 * @return whether the limit is reached
	 */
	private boolean isRightLimitSwitchReached() {
		return elevatorRightLimitSwitch.get();
	}

	/**
	 * Deadbanded elevator command. Single source of truth for both
	 * state transitions and motor output.
	 *
	 * @param input Global TeleopInput
	 * @return elevator command in [-1, 1]
	 */
	private double getElevatorInput(TeleopInput input) {
		return MathUtil.applyDeadband(input.getLeftJoystickX(),
				ElevatorConstants.JOYSTICK_DEADBAND);
	}

	/**
	 * Deadbanded pivot command. Single source of truth for both
	 * state transitions and motor output.
	 *
	 * @param input Global TeleopInput
	 * @return pivot command in [-1, 1]
	 */
	private double getPivotInput(TeleopInput input) {
		return MathUtil.applyDeadband(input.getRightJoystickX(),
				ElevatorConstants.JOYSTICK_DEADBAND);
	}

	private boolean isPivoting(TeleopInput input) {
		return input.getPivotLeftButton() || input.getPivotRightButton();
	}

	private boolean isElevating(TeleopInput input) {
		return input.getElevateUpButton() || input.getElevateDownButton();
	}

	/**
	 * Zero the encoder at the bottom limit and block motion past either elevator
	 * limit.
	 *
	 * @param signalInput requested elevator command
	 * @return elevator command after limits are applied
	 */
	private double applyElevatorLimits(double signalInput) {
		if (isBottomLimitReached()) {
			elevatorMotorUp1.setPosition(0);
			if (signalInput < 0) {
				return 0;
			}
		}
		if (isUpperLimitReached() && signalInput > 0) {
			return 0;
		}
		return signalInput;
	}

	/**
	 * Block pivot motion past either pivot limit.
	 *
	 * @param signalInput requested pivot command
	 * @return pivot command after limits are applied
	 */
	private double applyPivotLimits(double signalInput) {
		// Assumes positive = toward the right limit switch; flip if your mechanism is
		// the opposite
		if (isRightLimitSwitchReached() && signalInput > 0) {
			return 0;
		}
		if (isLeftLimitSwitchReached() && signalInput < 0) {
			return 0;
		}
		return signalInput;
	}

	private double getElevatorPos() {
		return elevatorMotorUp1.getPosition().getValueAsDouble();
	}

	private double getPivotPos() {
		return elevatorMotorPivot1.getPosition().getValueAsDouble();
	}

	private void holdElevator() {
		elevatorMotorUp1.setControl(elevatorHoldRequest.withPosition(elevatorHoldTarget));
	}

	private void holdPivot() {
		elevatorMotorPivot1.setControl(pivotHoldRequest.withPosition(pivotHoldTarget));
	}

	/* ------------------------ FSM state handlers ------------------------ */

	/**
	 * HOLD: both mechanisms servo to their last captured position.
	 *
	 * @param input the input received from teleop input
	 */
	private void handleHoldState(TeleopInput input) {
		holdElevator();
		holdPivot();
	}

	/**
	 * ELEVATE: up/down buttons drive the elevator at constant speed, pivot is held
	 * in place.
	 *
	 * @param input the input received from teleop input.
	 */
	private void handleElevateState(TeleopInput input) {
		double speed = 0;
		if (input.getElevateUpButton() && !input.getElevateDownButton()) {
			speed = ElevatorConstants.ELEVATOR_MANUAL_SCALE;
		} else if (input.getElevateDownButton() && !input.getElevateUpButton()) {
			speed = -ElevatorConstants.ELEVATOR_MANUAL_SCALE;
		}

		speed = applyElevatorLimits(speed);

		if (speed == 0) {
			holdElevator();
		} else {
			elevatorMotorUp1.set(speed);
			elevatorHoldTarget = getElevatorPos(); // keep target following us while moving
		}

		holdPivot();
	}

	/**
	 * PIVOT: left/right buttons drive the pivot at constant speed, elevator is held
	 * in place.
	 *
	 * @param input the input received from teleop input.
	 */
	private void handlePivotState(TeleopInput input) {
		double speed = 0;
		if (input.getPivotRightButton() && !input.getPivotLeftButton()) {
			speed = ElevatorConstants.PIVOT_MANUAL_SCALE;
		} else if (input.getPivotLeftButton() && !input.getPivotRightButton()) {
			speed = -ElevatorConstants.PIVOT_MANUAL_SCALE;
		}

		speed = applyPivotLimits(speed);

		if (speed == 0) {
			holdPivot();
		} else {
			elevatorMotorPivot1.set(speed);
			pivotHoldTarget = getPivotPos();
		}

		holdElevator();
	}

	/**
	 * Is elevator at ground boolean accessor.
	 *
	 * @return whether or not elevator is at ground.
	 */
	public boolean isElevatorAtGround() {
		return isBottomLimitReached();
	}
}
