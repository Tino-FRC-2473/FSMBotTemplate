package frc.robot.input;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.input.InputTypes.ButtonInput;
import frc.robot.systems.DrivetrainFSMSystem;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class TeleopInputTest {
	private TeleopInput input;

	@BeforeEach
	void setup() {
		assertTrue(HAL.initialize(500, 0));
		DriverStationSim.resetData();
		connect(0, true);
		connect(1, true);
		DriverStationSim.notifyNewData();
		input = new TeleopInput();
		input.synchronize();
	}

	private static void connect(int port, boolean connected) {
		DriverStationSim.setJoystickAxisCount(port, connected ? 6 : 0);
		DriverStationSim.setJoystickButtonCount(port, connected ? 10 : 0);
		DriverStationSim.setJoystickPOVCount(port, connected ? 1 : 0);
		if (connected) {
			DriverStationSim.setJoystickPOV(port, 0, -1);
		}
	}

	private void poll() {
		DriverStationSim.notifyNewData();
		input.poll();
	}

	@Test
	void allConsumersSeeTheSameEdgesAndGettersNeverResample() {
		DriverStationSim.setJoystickButton(1, 2, true);
		poll();
		for (int read = 0; read < 5; read++) {
			assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
			assertTrue(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		}
		DriverStationSim.setJoystickButton(1, 2, false);
		DriverStationSim.notifyNewData();
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		input.poll();
		assertTrue(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
		assertTrue(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		poll();
		assertFalse(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
	}

	@Test
	void holdingButtonsDoesNotRepeatPressEdges() {
		DriverStationSim.setJoystickButton(0, 8, true);
		DriverStationSim.setJoystickButton(1, 2, true);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED));
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		poll();
		assertTrue(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
	}

	@Test
	void teleopEntryBlocksHeldButtonsUntilReleasedWithoutFakeRelease() {
		DriverStationSim.setJoystickButton(1, 2, true);
		DriverStationSim.notifyNewData();
		input.synchronize();
		poll();
		assertFalse(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
		DriverStationSim.setJoystickButton(1, 2, false);
		poll();
		assertFalse(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
		DriverStationSim.setJoystickButton(1, 2, true);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
	}

	@Test
	void disconnectClearsValuesWithoutReleaseAndReconnectRequiresRelease() {
		DriverStationSim.setJoystickButton(1, 2, true);
		DriverStationSim.setJoystickAxis(1, 0, 0.7);
		DriverStationSim.setJoystickAxis(0, 1, -0.8);
		poll();
		connect(1, false);
		poll();
		assertFalse(input.isMechanismControllerConnected());
		assertTrue(input.isDriverControllerConnected());
		assertFalse(input.getButtonReleased(ButtonInput.INTAKE_BUTTON));
		assertFalse(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		assertEquals(0.0, input.getAxisValue(AxialInput.CLIMBER_MANUAL_CONTROL));
		assertEquals(0.8, input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X), 1e-6);
		connect(1, true);
		poll();
		assertFalse(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		DriverStationSim.setJoystickButton(1, 2, false);
		poll();
		DriverStationSim.setJoystickButton(1, 2, true);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.INTAKE_BUTTON));
	}

	@Test
	void driverDisconnectStopsLogicalDriveWithoutAffectingMechanism() {
		DriverStationSim.setJoystickAxis(0, 1, -1.0);
		DriverStationSim.setJoystickButton(0, 8, true);
		DriverStationSim.setJoystickButton(1, 2, true);
		poll();
		connect(0, false);
		poll();
		assertFalse(input.isDriverControllerConnected());
		assertFalse(input.getButtonReleased(ButtonInput.DRIVETRAIN_RESEED));
		assertTrue(input.getButtonValue(ButtonInput.INTAKE_BUTTON));
		assertEquals(new DrivetrainFSMSystem.DriveInput(0.0, 0.0, 0.0),
			DrivetrainFSMSystem.mapXboxInput(input));
		connect(0, true);
		poll();
		assertEquals(1.0, input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X));
		assertFalse(input.getButtonPressed(ButtonInput.DRIVETRAIN_RESEED));
	}

	@Test
	void mapsAxesWithExistingSignsAndDoesNotApplyAnotherDeadband() {
		DriverStationSim.setJoystickAxis(0, 0, -0.4);
		DriverStationSim.setJoystickAxis(0, 1, -0.8);
		DriverStationSim.setJoystickAxis(0, 4, -0.6);
		DriverStationSim.setJoystickAxis(1, 0, 0.05);
		poll();
		var demand = DrivetrainFSMSystem.mapXboxInput(input);
		assertEquals(0.8, demand.forward(), 1e-6);
		assertEquals(0.4, demand.left(), 1e-6);
		assertEquals(0.6, demand.rotation(), 1e-6);
		assertEquals(0.05, input.getAxisValue(AxialInput.CLIMBER_MANUAL_CONTROL), 1e-6);
	}

	@Test
	void snapshotClampsAxesRejectsNonfiniteAndNullKeys() {
		input.sampleAxis(AxialInput.DRIVETRAIN_DRIVE_X, 2.0);
		assertEquals(1.0, input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X));
		input.sampleAxis(AxialInput.DRIVETRAIN_DRIVE_X, -2.0);
		assertEquals(-1.0, input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X));
		for (double invalid : new double[] {Double.NaN, Double.POSITIVE_INFINITY}) {
			input.sampleAxis(AxialInput.DRIVETRAIN_DRIVE_X, invalid);
			assertEquals(0.0, input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X));
		}
		assertThrows(NullPointerException.class, () -> input.getAxisValue(null));
		assertThrows(NullPointerException.class, () -> input.getButtonValue(null));
		assertThrows(NullPointerException.class, () -> input.getButtonPressed(null));
		assertThrows(NullPointerException.class, () -> input.getButtonReleased(null));
	}

	@Test
	void allDigitalBindingsMatchTheSeasonControls() {
		Map<ButtonInput, Integer> driverButtons = Map.of(
			ButtonInput.DRIVETRAIN_RESEED, 8, ButtonInput.DRIVETRAIN_PATHFIND, 4,
			ButtonInput.DRIVETRAIN_INVERT_CONTROLS, 7, ButtonInput.BALL_SHAKE_FRONT, 3,
			ButtonInput.BALL_SHAKE_SIDE, 1);
		Map<ButtonInput, Integer> mechanismButtons = Map.of(
			ButtonInput.INTAKE_BUTTON, 2, ButtonInput.OUTTAKE_BUTTON, 1,
			ButtonInput.FOLD_IN_BUTTON, 4, ButtonInput.FOLD_OUT_BUTTON, 3,
			ButtonInput.PARTIAL_OUT_BUTTON, 8, ButtonInput.PASSER_PREP_TOGGLE, 6,
			ButtonInput.MANUAL_SHOOT_TOGGLE, 5, ButtonInput.IDLE_SHOOTER_TOGGLE, 7);
		for (var entry : driverButtons.entrySet()) {
			DriverStationSim.setJoystickButton(0, entry.getValue(), true);
			poll();
			assertTrue(input.getButtonPressed(entry.getKey()), entry.getKey().name());
			DriverStationSim.setJoystickButton(0, entry.getValue(), false);
			poll();
			assertTrue(input.getButtonReleased(entry.getKey()), entry.getKey().name());
		}
		for (var entry : mechanismButtons.entrySet()) {
			DriverStationSim.setJoystickButton(1, entry.getValue(), true);
			poll();
			assertTrue(input.getButtonPressed(entry.getKey()), entry.getKey().name());
			DriverStationSim.setJoystickButton(1, entry.getValue(), false);
			poll();
			assertTrue(input.getButtonReleased(entry.getKey()), entry.getKey().name());
		}
		DriverStationSim.setJoystickAxis(0, 2, 0.8);
		DriverStationSim.setJoystickAxis(1, 3, 0.8);
		DriverStationSim.setJoystickPOV(1, 0, 0);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.FACE_PASS));
		assertTrue(input.getButtonPressed(ButtonInput.REV_FEEDER));
		assertTrue(input.getButtonPressed(ButtonInput.CLIMBER_MANUAL_OVERRIDE));
		DriverStationSim.setJoystickPOV(1, 0, 270);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.CLIMBER_NEXT_STEP));
	}

	@Test
	void triggersPovAndReservedBindingsAreExplicit() {
		DriverStationSim.setJoystickAxis(0, 3, 0.5);
		poll();
		assertFalse(input.getButtonValue(ButtonInput.FACE_HUB));
		DriverStationSim.setJoystickAxis(0, 3, 0.6);
		DriverStationSim.setJoystickAxis(1, 2, 0.7);
		DriverStationSim.setJoystickPOV(1, 0, 180);
		poll();
		assertTrue(input.getButtonPressed(ButtonInput.FACE_HUB));
		assertTrue(input.getButtonPressed(ButtonInput.SHOOTER_PREP_TOGGLE));
		assertTrue(input.getButtonPressed(ButtonInput.CLIMBER_EMERGENCY_ABORT));
		assertFalse(input.getButtonValue(ButtonInput.CLIMBER_AUTO_UP_1));
		assertFalse(input.getButtonValue(ButtonInput.CLIMBER_AUTO_UP_2));
	}
}
