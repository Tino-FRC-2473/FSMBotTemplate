package frc.robot.systems;

import java.util.Objects;
import java.util.function.Function;

import edu.wpi.first.math.MathUtil;
import frc.robot.TeleopInput;
import frc.robot.TeleopInput.DriveAxes;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.systems.AutoHandlerSystem.AutoFSMState;

/**
 * Generic manually ticked field-centric swerve FSM. Driver axes are normalized in the input
 * abstraction; per-axis deadband, speed scaling, and all stop behavior are handled here.
 */
public class DrivetrainFSMSystem extends FSMSystem<DrivetrainFSMSystem.DriveState> {
	public enum DriveState {
		STOPPED,
		DRIVING
	}

	/** Normalized chassis demand, with each component expected in the range [-1, 1]. */
	public record DriveInput(double forward, double left, double rotation) { }

	/** Converts operator input to normalized chassis demand. */
	@FunctionalInterface
	public interface InputMapper extends Function<TeleopInput, DriveInput> {
		@Override
		DriveInput apply(TeleopInput input);
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
			if (!Double.isFinite(maxAngularSpeedRadiansPerSecond) || maxAngularSpeedRadiansPerSecond <= 0.0) {
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
	public void update(TeleopInput input) {
		drivetrain.updateOperatorPerspective();
		if (input == null || !input.isDriverControllerConnected()) {
			stop();
			return;
		}

		DriveInput demand = inputMapper.apply(input);
		if (demand == null) {
			stop();
			return;
		}

		double forward = applyDeadbandAndClamp(demand.forward(), config.deadband());
		double left = applyDeadbandAndClamp(demand.left(), config.deadband());
		double rotation = applyDeadbandAndClamp(demand.rotation(), config.deadband());
		drivetrain.driveFieldCentric(
			forward * config.maxLinearSpeedMetersPerSecond(),
			left * config.maxLinearSpeedMetersPerSecond(),
			rotation * config.maxAngularSpeedRadiansPerSecond());
		setCurrentState(DriveState.DRIVING);
	}

	/** Convenience mapper for the built-in Xbox input abstraction. */
	public static DriveInput mapXboxInput(TeleopInput input) {
		if (input == null || !input.isDriverControllerConnected()) {
			return new DriveInput(0.0, 0.0, 0.0);
		}
		DriveAxes axes = input.getDriveAxes();
		return new DriveInput(axes.forward(), axes.left(), axes.counterClockwiseRotation());
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
	public boolean updateAutonomous(AutoFSMState autoState) {
		// Path following is intentionally not part of this generic input FSM.
		stop();
		return true;
	}

	@Override
	protected DriveState nextState(TeleopInput input) {
		return input == null || !input.isDriverControllerConnected() ? DriveState.STOPPED : DriveState.DRIVING;
	}

	private void stop() {
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
