package frc.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.input.TeleopInput;
import frc.robot.systems.FSMSystem;
import frc.robot.systems.PathPlannerAutoSystem;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class RobotMechanismLifecycleTest {
	private enum State { STOPPED, RUNNING }

	private static class RecordingFSM extends FSMSystem<State> {
		private int updates;
		private int resets;
		private Input lastInput;
		private boolean intakePressed;
		private State appliedState = State.STOPPED;
		private boolean failUpdate;
		private boolean failReset;

		RecordingFSM() {
			setState(State.STOPPED);
		}

		@Override
		public void reset() {
			resets++;
			setState(State.STOPPED);
			appliedState = State.STOPPED;
			if (failReset) {
				throw new IllegalStateException("Test reset failure");
			}
		}

		@Override
		public void update(Input input) {
			updates++;
			lastInput = input;
			intakePressed = input != null && input.getButtonPressed(ButtonInput.INTAKE_BUTTON);
			appliedState = getCurrentState();
			if (failUpdate) {
				throw new IllegalStateException("Test update failure");
			}
		}

		@Override
		protected State nextState(Input input) {
			return getCurrentState();
		}
	}

	private Robot robot;
	private SendableChooser<Command> chooser;
	private PathPlannerAutoSystem auto;
	private RecordingFSM mechanism;
	private TeleopInput input;

	@BeforeEach
	void setup() {
		assertTrue(HAL.initialize(500, 0));
		SimHooks.pauseTiming();
		DriverStationSim.resetData();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setJoystickAxisCount(0, 6);
		DriverStationSim.setJoystickButtonCount(0, 10);
		DriverStationSim.setJoystickAxisCount(1, 6);
		DriverStationSim.setJoystickButtonCount(1, 10);
		DriverStationSim.setAutonomous(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		CommandScheduler.getInstance().enable();
		chooser = new SendableChooser<>();
		AtomicReference<Robot> owner = new AtomicReference<>();
		auto = new PathPlannerAutoSystem(chooser, () -> { }, () -> owner.get().resetMechanisms());
		input = new TeleopInput();
		robot = new Robot(auto, input);
		NetworkTableInstance.getDefault().stopServer();
		owner.set(robot);
		mechanism = new RecordingFSM();
		robot.registerMechanism(mechanism);
	}

	@AfterEach
	void cleanup() {
		robot.close();
		chooser.close();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		SimHooks.resumeTiming();
	}

	private Command selectRunningAuto() {
		Command routine = Commands.sequence(mechanism.setStateCommand(State.RUNNING),
			Commands.waitSeconds(10.0));
		chooser.setDefaultOption("Mechanism Auto", routine);
		robot.autonomousInit();
		return routine;
	}

	@Test
	void initializationHookRunsWithoutDriveHardwareAndClosesRegistration() {
		RecordingFSM initialized = new RecordingFSM();
		try (Robot template = new Robot() {
			@Override
			protected void initializeMechanisms() {
				registerMechanism(initialized);
			}
		}) {
			template.robotInit();
			assertThrows(IllegalStateException.class,
				() -> template.registerMechanism(new RecordingFSM()));
			template.teleopPeriodic();
			assertEquals(1, initialized.updates);
			assertTrue(initialized.lastInput != null);
		}
		assertEquals(State.STOPPED, initialized.appliedState);
	}

	@Test
	void teleopPollsBeforeConsumersAndOperatorLossResetsOnlyOnceUntilReconnect() {
		robot.teleopInit();
		DriverStationSim.setJoystickButton(1, 2, true);
		DriverStationSim.notifyNewData();
		robot.teleopPeriodic();
		assertTrue(mechanism.intakePressed);
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		mechanism.setState(State.RUNNING);
		robot.teleopPeriodic();
		assertEquals(State.RUNNING, mechanism.appliedState);
		int updates = mechanism.updates;
		int resets = mechanism.resets;
		DriverStationSim.setJoystickAxisCount(1, 0);
		DriverStationSim.setJoystickButtonCount(1, 0);
		DriverStationSim.notifyNewData();
		robot.teleopPeriodic();
		assertEquals(State.STOPPED, mechanism.appliedState);
		assertEquals(resets + 1, mechanism.resets);
		assertEquals(updates, mechanism.updates);
		robot.teleopPeriodic();
		assertEquals(resets + 1, mechanism.resets);
		assertTrue(input.isDriverControllerConnected());
		DriverStationSim.setJoystickAxisCount(1, 6);
		DriverStationSim.setJoystickButtonCount(1, 10);
		DriverStationSim.notifyNewData();
		robot.teleopPeriodic();
		assertEquals(updates + 1, mechanism.updates);
		assertFalse(mechanism.intakePressed);
	}

	@Test
	void driverLossDoesNotDisableOperatorAndAutoNeverPollsControllerInput() {
		robot.teleopInit();
		DriverStationSim.setJoystickAxisCount(0, 0);
		DriverStationSim.setJoystickButtonCount(0, 0);
		DriverStationSim.setJoystickButton(1, 2, true);
		DriverStationSim.notifyNewData();
		robot.teleopPeriodic();
		assertTrue(mechanism.intakePressed);
		assertFalse(input.isDriverControllerConnected());
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		DriverStationSim.setJoystickButton(1, 2, false);
		DriverStationSim.setJoystickAxisCount(1, 0);
		DriverStationSim.setJoystickButtonCount(1, 0);
		DriverStationSim.notifyNewData();
		selectRunningAuto();
		robot.autonomousPeriodic();
		assertNull(mechanism.lastInput);
		assertEquals(State.RUNNING, mechanism.appliedState);
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON),
			"Autonomous must not poll or consume the last teleop snapshot");
	}

	@Test
	void rejectsDuplicateAndNullMechanismRegistration() {
		assertThrows(IllegalArgumentException.class, () -> robot.registerMechanism(mechanism));
		assertThrows(NullPointerException.class, () -> robot.registerMechanism(null));
	}

	@Test
	void autonomousTicksOnceAfterCommandsAndTeleopUsesTheSameFsmWithControllerInput() {
		selectRunningAuto();
		assertEquals(State.RUNNING, mechanism.getCurrentState());
		assertEquals(0, mechanism.updates);
		robot.autonomousPeriodic();
		assertEquals(1, mechanism.updates);
		assertNull(mechanism.lastInput);
		assertEquals(State.RUNNING, mechanism.appliedState);
		robot.autonomousPeriodic();
		assertEquals(2, mechanism.updates);
		DriverStationSim.setAutonomous(false);
		DriverStationSim.notifyNewData();
		robot.teleopInit();
		assertEquals(State.STOPPED, mechanism.appliedState);
		robot.teleopPeriodic();
		assertEquals(3, mechanism.updates);
		assertSame(input, mechanism.lastInput);
	}

	@Test
	void completionAndTimeoutResetPersistentStateAndStopFurtherAutonomousTicks() {
		chooser.setDefaultOption("Short Auto", Commands.sequence(
			mechanism.setStateCommand(State.RUNNING), Commands.waitSeconds(0.1)));
		robot.autonomousInit();
		robot.autonomousPeriodic();
		SimHooks.stepTimingAsync(0.2);
		robot.autonomousPeriodic();
		assertFalse(auto.isRunning());
		assertEquals(State.STOPPED, mechanism.appliedState);
		int updates = mechanism.updates;
		robot.autonomousPeriodic();
		assertEquals(updates, mechanism.updates);

		chooser.setDefaultOption("Long Auto", Commands.sequence(
			mechanism.setStateCommand(State.RUNNING), Commands.waitSeconds(20.0)));
		robot.autonomousInit();
		robot.autonomousPeriodic();
		SimHooks.stepTimingAsync(15.01);
		robot.autonomousPeriodic();
		assertFalse(auto.isRunning());
		assertEquals(State.STOPPED, mechanism.appliedState);
	}

	@Test
	void setterOnlyAutoFinishesAndResetsWithoutExecutingMechanismOutputs() {
		chooser.setDefaultOption("Set Only", mechanism.setStateCommand(State.RUNNING));
		robot.autonomousInit();
		robot.autonomousPeriodic();
		assertEquals(0, mechanism.updates);
		assertEquals(State.STOPPED, mechanism.appliedState);
	}

	@Test
	void disableTestTeleopAndShutdownResetEvenWhenOneShotCommandsHaveFinished() {
		for (Runnable modeChange : List.<Runnable>of(robot::disabledInit, robot::testInit,
			robot::teleopInit, robot::close)) {
			selectRunningAuto();
			robot.autonomousPeriodic();
			assertEquals(State.RUNNING, mechanism.appliedState);
			modeChange.run();
			assertFalse(auto.isRunning());
			assertEquals(State.STOPPED, mechanism.appliedState);
		}
	}

	@Test
	void mechanismsResetWhenPathPlannerIsUnavailable() {
		try (Robot unavailable = new Robot(null, input)) {
			RecordingFSM fsm = new RecordingFSM();
			unavailable.registerMechanism(fsm);
			fsm.setState(State.RUNNING);
			fsm.update(null);
			unavailable.disabledInit();
			assertEquals(State.STOPPED, fsm.appliedState);
			unavailable.autonomousPeriodic();
			assertEquals(1, fsm.updates);
		}
	}

	@Test
	void cancellationAndRepeatedAutoStartClearStaleStates() {
		Command routine = selectRunningAuto();
		robot.autonomousPeriodic();
		auto.cancel();
		assertFalse(routine.isScheduled());
		assertEquals(State.STOPPED, mechanism.appliedState);
		robot.autonomousInit();
		assertEquals(State.RUNNING, mechanism.getCurrentState());
		assertEquals(State.STOPPED, mechanism.appliedState);
		robot.autonomousPeriodic();
		assertEquals(State.RUNNING, mechanism.appliedState);
	}

	@Test
	void mechanismFailureCancelsAutoResetsEveryMechanismAndSuppressesLaterTicks() {
		RecordingFSM second = new RecordingFSM();
		robot.registerMechanism(second);
		selectRunningAuto();
		mechanism.failUpdate = true;
		robot.autonomousPeriodic();
		assertFalse(auto.isRunning());
		assertEquals(State.STOPPED, mechanism.appliedState);
		assertEquals(State.STOPPED, second.appliedState);
		assertTrue(second.resets > 0);
		int updates = mechanism.updates;
		robot.autonomousPeriodic();
		robot.teleopPeriodic();
		assertEquals(updates, mechanism.updates);
	}

	@Test
	void resetFailureDoesNotSkipOtherMechanismsAndPreventsStartingAuto() {
		RecordingFSM second = new RecordingFSM();
		robot.registerMechanism(second);
		mechanism.failReset = true;
		Command routine = selectRunningAuto();
		assertFalse(routine.isScheduled());
		assertTrue(second.resets > 0);
		assertEquals(State.STOPPED, second.appliedState);
		mechanism.failReset = false;
	}
}
