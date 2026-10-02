package frc.robot.systems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.input.Input;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class FSMSystemTest {
	private enum State { STOPPED, RUNNING, DONE }

	private static class RecordingFSM extends FSMSystem<State> {
		private int assignments;
		private int updates;
		private boolean sensorFinished;
		private State appliedState;

		RecordingFSM() {
			reset();
		}

		@Override
		public void setState(State state) {
			super.setState(state);
			assignments++;
		}

		@Override
		public void reset() {
			setCurrentState(State.STOPPED);
			appliedState = State.STOPPED;
		}

		@Override
		public void update(Input input) {
			updates++;
			appliedState = getCurrentState();
			setCurrentState(nextState(input));
		}

		@Override
		protected State nextState(Input input) {
			return sensorFinished ? State.DONE : getCurrentState();
		}
	}

	@BeforeEach
	void enableAutonomous() {
		assertTrue(HAL.initialize(500, 0));
		DriverStationSim.resetData();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		CommandScheduler.getInstance().enable();
	}

	@AfterEach
	void cleanup() {
		CommandScheduler.getInstance().cancelAll();
		NamedCommands.clearAll();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
	}

	@Test
	void directlySetsStateWithoutApplyingOutputsOrPreventingNormalTransitions() {
		RecordingFSM fsm = new RecordingFSM();
		fsm.setState(State.RUNNING);
		assertEquals(State.RUNNING, fsm.getCurrentState());
		assertEquals(State.STOPPED, fsm.appliedState);
		fsm.update(null);
		assertEquals(State.RUNNING, fsm.appliedState);
		fsm.sensorFinished = true;
		fsm.update(null);
		assertEquals(State.DONE, fsm.getCurrentState());
	}

	@Test
	void rejectsNullAndAllowsRepeatedStateAssignments() {
		RecordingFSM fsm = new RecordingFSM();
		assertThrows(NullPointerException.class, () -> fsm.setState(null));
		assertThrows(NullPointerException.class, () -> fsm.setStateCommand(null));
		fsm.setState(State.RUNNING);
		fsm.setState(State.RUNNING);
		assertEquals(3, fsm.assignments);
	}

	@Test
	void commandSetsStateOnceOnlyWhenInitializedAndNeverTicksTheFsm() {
		RecordingFSM fsm = new RecordingFSM();
		Command command = fsm.setStateCommand(State.RUNNING);
		assertEquals(State.STOPPED, fsm.getCurrentState());
		assertTrue(command.getRequirements().contains(fsm));
		CommandScheduler.getInstance().schedule(command);
		assertEquals(State.RUNNING, fsm.getCurrentState());
		assertEquals(command, CommandScheduler.getInstance().requiring(fsm));
		CommandScheduler.getInstance().run();
		assertFalse(command.isScheduled());
		assertNull(CommandScheduler.getInstance().requiring(fsm));
		assertEquals(2, fsm.assignments);
		assertEquals(0, fsm.updates);
		assertEquals(State.RUNNING, fsm.getCurrentState());
		fsm.reset();
		CommandScheduler.getInstance().schedule(command);
		CommandScheduler.getInstance().run();
		assertEquals(State.RUNNING, fsm.getCurrentState());
		assertEquals(4, fsm.assignments);
	}

	@Test
	void namedCommandCanBeTriggeredRepeatedlyWithTheSameMechanismRequirement() {
		RecordingFSM fsm = new RecordingFSM();
		NamedCommands.registerCommand("Test/Run", fsm.setStateCommand(State.RUNNING));
		for (int repetition = 0; repetition < 2; repetition++) {
			fsm.reset();
			Command event = NamedCommands.getCommand("Test/Run");
			assertTrue(event.getRequirements().contains(fsm));
			CommandScheduler.getInstance().schedule(event);
			CommandScheduler.getInstance().run();
			assertEquals(State.RUNNING, fsm.getCurrentState());
			assertFalse(event.isScheduled());
		}
		assertEquals(0, fsm.updates);
	}
}
