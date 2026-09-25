package frc.robot.generated;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Nonfunctional API scaffold for the 2026 Phoenix Tuner X generated file.
 *
 * <p>Replace this entire file with this robot's Phoenix Tuner X output before deploying.
 * No hardware IDs, CAN bus, module geometry, offsets, or gains are supplied here. The
 * Tuner X output must keep the package {@code frc.robot.generated} and use the template's
 * {@code frc.robot.subsystems.CommandSwerveDrivetrain} wrapper.
 */
public final class TunerConstants {
	private TunerConstants() { }

	/** Placeholder only; the generated file supplies the measured robot speed. */
	public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(0.0);

	/** Placeholder fails safely until real robot-specific Tuner output replaces this file. */
	public static CommandSwerveDrivetrain createDrivetrain() {
		throw new UnsupportedOperationException(
			"Replace this TunerConstants scaffold with Phoenix Tuner X output before using the drivetrain.");
	}

	/**
	 * Source-compatible stand-in for the nested drivetrain class emitted by Tuner X.
	 * It contains no robot configuration and must not be used to construct hardware.
	 */
	public static class TunerSwerveDrivetrain extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> {
		public TunerSwerveDrivetrain(
			SwerveDrivetrainConstants drivetrainConstants,
			SwerveModuleConstants<?, ?, ?>... modules) {
			super(TalonFX::new, TalonFX::new, CANcoder::new, drivetrainConstants, modules);
		}
	}
}
