package frc.robot.systems;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.input.Input;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class PathPlannerAutoSystemTest {
	private static final Translation2d[] MODULES = {
		new Translation2d(0.3, 0.3), new Translation2d(0.3, -0.3),
		new Translation2d(-0.3, 0.3), new Translation2d(-0.3, -0.3)
	};

	private final AtomicInteger stops = new AtomicInteger();
	private SendableChooser<Command> chooser;
	private PathPlannerAutoSystem system;

	@BeforeAll
	static void initializeHal() {
		assertTrue(HAL.initialize(500, 0));
	}

	@BeforeEach
	void prepareAutonomous() {
		SimHooks.pauseTiming();
		DriverStationSim.resetData();
		DriverStationSim.setDsAttached(true);
		DriverStationSim.setAutonomous(true);
		DriverStationSim.setEnabled(true);
		DriverStationSim.notifyNewData();
		CommandScheduler.getInstance().enable();
		chooser = new SendableChooser<>();
		system = new PathPlannerAutoSystem(chooser, stops::incrementAndGet);
	}

	@AfterEach
	void cleanup() {
		system.cancel();
		chooser.close();
		NamedCommands.clearAll();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();
		SimHooks.resumeTiming();
	}

	@Test
	void schedulesSelectionWithoutOverwritingRunningDriveOutput() {
		AtomicInteger executions = new AtomicInteger();
		Command command = Commands.run(executions::incrementAndGet);
		chooser.setDefaultOption("Drive", command);

		system.start();
		assertTrue(command.isScheduled());
		int stopsBeforeUpdate = stops.get();
		system.update();
		system.update();

		assertEquals(2, executions.get());
		assertEquals(stopsBeforeUpdate, stops.get());
	}

	@Test
	void completionStopsAndDoesNotRestartTheRoutine() {
		AtomicInteger executions = new AtomicInteger();
		Command command = Commands.runOnce(executions::incrementAndGet);
		chooser.setDefaultOption("Finish", command);

		system.start();
		int stopsBeforeUpdate = stops.get();
		system.update();
		assertFalse(command.isScheduled());
		assertTrue(stops.get() > stopsBeforeUpdate);
		system.update();
		assertEquals(1, executions.get());
	}

	@Test
	void cancellationStopsRoutineAndSeparatelyScheduledEventCommands() {
		AtomicInteger interruptions = new AtomicInteger();
		Command command = Commands.run(() -> { }).finallyDo(interrupted -> {
			if (interrupted) {
				interruptions.incrementAndGet();
			}
		});
		Command eventCommand = Commands.run(() -> { });
		chooser.setDefaultOption("Drive", command);
		system.start();
		CommandScheduler.getInstance().schedule(eventCommand);
		int stopsBeforeCancel = stops.get();

		system.cancel();

		assertFalse(command.isScheduled());
		assertFalse(eventCommand.isScheduled());
		assertEquals(1, interruptions.get());
		assertTrue(stops.get() > stopsBeforeCancel);
	}

	@Test
	void disablingInterruptsAndStopsTheRoutine() {
		Command command = Commands.run(() -> { });
		chooser.setDefaultOption("Drive", command);
		system.start();
		DriverStationSim.setEnabled(false);
		DriverStationSim.notifyNewData();

		system.update();

		assertFalse(command.isScheduled());
		assertTrue(stops.get() >= 2);
	}

	@Test
	void samePreloadedRoutineCanRunAgainAfterCancellation() {
		AtomicInteger executions = new AtomicInteger();
		Command command = Commands.run(executions::incrementAndGet);
		chooser.setDefaultOption("Drive", command);
		system.start();
		system.update();
		system.cancel();
		system.start();
		system.update();

		assertTrue(command.isScheduled());
		assertEquals(2, executions.get());
	}

	@Test
	void initializationFailureCanBeCancelledWithoutLeavingDriveActive() {
		AtomicInteger mechanismResets = new AtomicInteger();
		system = new PathPlannerAutoSystem(chooser, stops::incrementAndGet,
			mechanismResets::incrementAndGet);
		Command command = new Command() {
			@Override
			public void initialize() {
				throw new IllegalStateException("Test initialization failure");
			}
		};
		chooser.setDefaultOption("Failing Start", command);

		assertThrows(IllegalStateException.class, system::start);
		int stopsBeforeCancel = stops.get();
		assertDoesNotThrow(system::cancel);

		assertFalse(command.isScheduled());
		assertTrue(stops.get() > stopsBeforeCancel);
		assertTrue(mechanismResets.get() >= 3);
	}

	@Test
	void runtimeFailureResetsMechanismsAndPreventsRetry() {
		AtomicInteger resets = new AtomicInteger();
		system = new PathPlannerAutoSystem(chooser, stops::incrementAndGet, resets::incrementAndGet);
		Command routine = Commands.run(() -> { });
		chooser.setDefaultOption("Drive", routine);
		Subsystem failingPeriodic = new Subsystem() {
			@Override
			public void periodic() {
				throw new IllegalStateException("Test scheduler runtime failure");
			}
		};
		CommandScheduler.getInstance().registerSubsystem(failingPeriodic);
		try {
			system.start();
			int resetsBeforeUpdate = resets.get();
			assertDoesNotThrow(system::update);
			assertFalse(system.isRunning());
			assertFalse(routine.isScheduled());
			assertTrue(resets.get() > resetsBeforeUpdate);
			system.start();
			assertFalse(routine.isScheduled());
		} finally {
			CommandScheduler.getInstance().unregisterSubsystem(failingPeriodic);
		}
	}

	@Test
	void deadlineCancelsAnUnfinishedRoutine() {
		Command command = Commands.run(() -> { });
		chooser.setDefaultOption("Long Auto", command);
		system.start();
		SimHooks.stepTimingAsync(15.01);

		system.update();

		assertFalse(command.isScheduled());
		assertTrue(stops.get() >= 2);
	}

	@Test
	void missingSelectionAndNoneAlwaysStop() {
		system.start();
		system.update();
		assertTrue(stops.get() >= 2);
		chooser.setDefaultOption("None", Commands.none());
		system.start();
		system.update();
		assertTrue(stops.get() >= 4);
	}

	@Test
	void flipsOnlyForKnownRedAlliance() {
		assertTrue(PathPlannerAutoSystem.shouldFlipPaths(Optional.of(Alliance.Red)));
		assertFalse(PathPlannerAutoSystem.shouldFlipPaths(Optional.of(Alliance.Blue)));
		assertFalse(PathPlannerAutoSystem.shouldFlipPaths(Optional.empty()));
	}

	@Test
	void acceptsMatchingGeometryAndRejectsIncorrectModuleOrder() {
		RobotConfig config = config(50.0);
		assertDoesNotThrow(() -> PathPlannerAutoSystem.validateConfig(config, MODULES));
		Translation2d[] reordered = {MODULES[1], MODULES[0], MODULES[2], MODULES[3]};
		assertThrows(IllegalArgumentException.class,
			() -> PathPlannerAutoSystem.validateConfig(config, reordered));
	}

	@Test
	void rejectsInvalidPhysicsAndDifferentialConfiguration() {
		assertThrows(IllegalArgumentException.class,
			() -> PathPlannerAutoSystem.validateConfig(config(Double.NaN), MODULES));
		RobotConfig differential = new RobotConfig(50.0, 6.0, config(50.0).moduleConfig, 0.6);
		assertThrows(IllegalArgumentException.class,
			() -> PathPlannerAutoSystem.validateConfig(differential, MODULES));
	}

	@Test
	void pathPlannerResetsPoseAndFollowsAPathWithRobotRelativeOutput() {
		AtomicReference<Pose2d> pose = new AtomicReference<>(new Pose2d());
		AtomicReference<ChassisSpeeds> speeds = new AtomicReference<>(new ChassisSpeeds());
		AtomicInteger outputCalls = new AtomicInteger();
		Subsystem drive = new Subsystem() { };
		AutoBuilder.configure(pose::get, pose::set, speeds::get, (output, feedforwards) -> {
			speeds.set(output);
			assertEquals(4, feedforwards.robotRelativeForcesXNewtons().length);
			assertEquals(4, feedforwards.robotRelativeForcesYNewtons().length);
			outputCalls.incrementAndGet();
		}, new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
			config(50.0), () -> false, drive);
		TestMechanism mechanism = new TestMechanism();
		NamedCommands.registerCommand("PathTest/Run", mechanism.setStateCommand(true));
		system = new PathPlannerAutoSystem(chooser, stops::incrementAndGet, mechanism::reset);
		Pose2d start = new Pose2d(1.0, 1.0, Rotation2d.kZero);
		PathPlannerPath path = new PathPlannerPath(
			PathPlannerPath.waypointsFromPoses(start, new Pose2d(2.0, 1.0, Rotation2d.kZero)),
			List.of(), List.of(), List.of(),
			List.of(new EventMarker("Run", 0.3, NamedCommands.getCommand("PathTest/Run"))),
			new PathConstraints(0.5, 0.5, 1.0, 1.0),
			new IdealStartingState(0.0, Rotation2d.kZero),
			new GoalEndState(0.0, Rotation2d.kZero), false);
		double duration = path.getIdealTrajectory(config(50.0)).orElseThrow().getTotalTimeSeconds();
		assertTrue(Double.isFinite(duration) && duration > 0.0, "Trajectory duration: " + duration);
		Command follow = AutoBuilder.followPath(path);
		assertTrue(follow.getRequirements().contains(drive));
		assertTrue(follow.getRequirements().contains(mechanism));
		Command auto = Commands.sequence(AutoBuilder.resetOdom(start), follow);
		chooser.setDefaultOption("Test Path", auto);

		system.start();
		assertEquals(start, pose.get());
		for (int step = 0; step < 500 && auto.isScheduled(); step++) {
			SimHooks.stepTimingAsync(0.02);
			system.update();
			if (system.isRunning()) {
				mechanism.update(null);
			}
			ChassisSpeeds demand = speeds.get();
			pose.set(pose.get().exp(new Twist2d(demand.vxMetersPerSecond * 0.02,
				demand.vyMetersPerSecond * 0.02, demand.omegaRadiansPerSecond * 0.02)));
		}

		assertTrue(mechanism.runningTicks > 0, "Named marker must run the mechanism during the path");
		assertFalse(mechanism.getCurrentState());
		assertTrue(outputCalls.get() > 10, "PathPlanner output calls: " + outputCalls.get());
		assertFalse(auto.isScheduled());
		assertEquals(2.0, pose.get().getX(), 0.1);
		assertEquals(1.0, pose.get().getY(), 0.05);
		assertTrue(stops.get() >= 2);
	}

	private static class TestMechanism extends FSMSystem<Boolean> {
		private int runningTicks;

		TestMechanism() {
			reset();
		}

		@Override
		public void reset() {
			setState(false);
		}

		@Override
		public void update(Input input) {
			assertEquals(null, input);
			if (getCurrentState()) {
				runningTicks++;
			}
		}

		@Override
		protected Boolean nextState(Input input) {
			return getCurrentState();
		}
	}

	@Test
	void mechanismCleanupRunsEvenIfDriveStopFailsAndAutoCannotRestart() {
		AtomicInteger resets = new AtomicInteger();
		system = new PathPlannerAutoSystem(chooser, () -> {
			throw new IllegalStateException("Test drive stop failure");
		}, resets::incrementAndGet);
		chooser.setDefaultOption("Drive", Commands.run(() -> { }));
		assertDoesNotThrow(system::start);
		assertFalse(system.isRunning());
		assertTrue(resets.get() > 0);
	}

	private static RobotConfig config(double mass) {
		ModuleConfig module = new ModuleConfig(0.05, 4.8, 1.0,
			DCMotor.getKrakenX60(1).withReduction(6.0), 60.0, 1);
		return new RobotConfig(mass, 6.0, module, MODULES);
	}
}
