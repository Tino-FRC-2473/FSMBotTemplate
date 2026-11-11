package frc.robot.systems;

import java.util.Objects;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import frc.robot.input.Input;
import frc.robot.input.InputTypes.AxialInput;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Generic manually ticked field-centric swerve FSM. Driver axes are normalized in the input
 * abstraction; per-axis deadband, speed scaling, and all stop behavior are handled here.
 */
public class DrivetrainFSMSystem extends FSMSystem<DrivetrainFSMSystem.DriveState> {
	/** Drive output state reported once per tick. */
	public enum DriveState {
		/** No usable controller or every axis inside the deadband; zero demand is commanded. */
		STOPPED,
		/** A nonzero chassis demand is being commanded. */
		DRIVING
	}

	/** Normalized chassis demand, with each component expected in the range [-1, 1]. */
	public record DriveInput(double forward, double left, double rotation) { }

	/** Converts operator input to normalized chassis demand. */
	@FunctionalInterface
	public interface InputMapper extends Function<Input, DriveInput> {
		@Override
		DriveInput apply(Input input);
	}

	/** Validated robot-provided drive limits and input deadband. */
	public record Config(
		double maxLinearSpeedMetersPerSecond,
		double maxAngularSpeedRadiansPerSecond,
		double deadband) {
		public Config {
			if (!Double.isFinite(maxLinearSpeedMetersPerSecond) || maxLinearSpeedMetersPerSecond <= 0.0) {
				throw new IllegalArgumentException("Maximum linear speed must be finite and positive");
			}
			if (!Double.isFinite(maxAngularSpeedRadiansPerSecond)
				|| maxAngularSpeedRadiansPerSecond <= 0.0) {
				throw new IllegalArgumentException("Maximum angular speed must be finite and positive");
			}
			if (!Double.isFinite(deadband) || deadband < 0.0 || deadband >= 1.0) {
				throw new IllegalArgumentException("Deadband must be in the range [0, 1)");
			}
		}
	}

	private final CommandSwerveDrivetrain drivetrain;
	private final Config config;
	private final InputMapper inputMapper;

	/**
	 * Create the drive FSM and immediately command a safe stopped state.
	 * @param drivetrain drivetrain that owns the module outputs
	 * @param config validated speed limits and deadband
	 * @param inputMapper converts the teleop snapshot to normalized chassis demand
	 */
	public DrivetrainFSMSystem(
		CommandSwerveDrivetrain drivetrain,
		Config config,
		InputMapper inputMapper) {
		this.drivetrain = Objects.requireNonNull(drivetrain, "drivetrain");
		this.config = Objects.requireNonNull(config, "config");
		this.inputMapper = Objects.requireNonNull(inputMapper, "inputMapper");
		reset();
	}

	@Override
	public void reset() {
		setCurrentState(DriveState.STOPPED);
		drivetrain.stop();
	}

	@Override
	public void update(Input input) {
		drivetrain.updateOperatorPerspective();
		if (nextState(input) == DriveState.STOPPED) {
			commandStop();
			return;
		}

		DriveInput demand = inputMapper.apply(input);
		if (demand == null) {
			commandStop();
			return;
		}

		double forward = applyDeadbandAndClamp(demand.forward(), config.deadband());
		double left = applyDeadbandAndClamp(demand.left(), config.deadband());
		double rotation = applyDeadbandAndClamp(demand.rotation(), config.deadband());
		if (forward == 0.0 && left == 0.0 && rotation == 0.0) {
			commandStop();
			return;
		}

		drivetrain.driveFieldCentric(
			forward * config.maxLinearSpeedMetersPerSecond(),
			left * config.maxLinearSpeedMetersPerSecond(),
			rotation * config.maxAngularSpeedRadiansPerSecond());
		setCurrentState(DriveState.DRIVING);
	}

	/** Convenience mapper for the built-in Xbox input abstraction. */
	public static DriveInput mapXboxInput(Input input) {
		if (input == null || !input.isDriverControllerConnected()) {
			return new DriveInput(0.0, 0.0, 0.0);
		}
		return new DriveInput(input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_X),
			input.getAxisValue(AxialInput.DRIVETRAIN_DRIVE_Y),
			input.getAxisValue(AxialInput.DRIVETRAIN_ROTATE));
	}

	/** Construct validated speed limits using Phoenix Tuner X measured speed and module radius. */
	public static Config configFromGeometry(
		double measuredMaxLinearSpeedMetersPerSecond,
		double maximumLinearSpeedScale,
		double maximumModuleRadiusMeters,
		double deadband) {
		if (!Double.isFinite(measuredMaxLinearSpeedMetersPerSecond)
			|| measuredMaxLinearSpeedMetersPerSecond <= 0.0) {
			throw new IllegalArgumentException("Generated maximum linear speed must be finite and positive");
		}
		if (!Double.isFinite(maximumLinearSpeedScale)
			|| maximumLinearSpeedScale <= 0.0 || maximumLinearSpeedScale > 1.0) {
			throw new IllegalArgumentException("Maximum linear speed scale must be in (0, 1]");
		}
		if (!Double.isFinite(maximumModuleRadiusMeters) || maximumModuleRadiusMeters <= 0.0) {
			throw new IllegalArgumentException("Generated module radius must be finite and positive");
		}

		double maxLinearSpeed = measuredMaxLinearSpeedMetersPerSecond * maximumLinearSpeedScale;
		return new Config(maxLinearSpeed, maxLinearSpeed / maximumModuleRadiusMeters, deadband);
	}

	@Override
	protected DriveState nextState(Input input) {
		return input == null || !input.isDriverControllerConnected()
			? DriveState.STOPPED : DriveState.DRIVING;
	}

	private void commandStop() {
		drivetrain.stop();
		setCurrentState(DriveState.STOPPED);
	}

	static double applyDeadbandAndClamp(double value, double deadband) {
		if (!Double.isFinite(value)) {
			return 0.0;
		}
		return MathUtil.applyDeadband(MathUtil.clamp(value, -1.0, 1.0), deadband);
	}
}
