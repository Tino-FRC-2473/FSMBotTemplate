package frc.robot.motors;

import java.util.Set;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.system.plant.DCMotor;

public class Constants {
	public static final String LOGGING_PREFIX = "Motor ";
	public static final Set<MotorType> ALLOWED_MOTOR_TYPES = Set.of(MotorType.kBrushless);
	public static final DCMotor DEFAULT_SPARK_CONFIG = DCMotor.getNeo550(1);
	public static final DCMotor DEFAULT_TALONFX_CONFIG = DCMotor.getKrakenX60(1);
}
