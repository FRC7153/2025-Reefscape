package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;

/**
 * Constants specific to the SwerveDrive base
 */
public final class SwerveConstants {
  // Speed configs
  public static final double MAX_WHEEL_VELOCITY = 5.0; // Absolute max wheel m/s

  // Max drive speeds
  public static final double FAST_TRANSLATIONAL_SPEED = 5.0; // m/s
  public static final double FAST_ROTATIONAL_SPEED = 3.0 * 2.0 * Math.PI; // rad/s

  public static final double SLOW_TRANSLATIONAL_SPEED = FAST_TRANSLATIONAL_SPEED * 0.5;
  public static final double SLOW_ROTATIONAL_SPEED = FAST_TRANSLATIONAL_SPEED * 0.5;

  //TODO
  // CANCoder magnet offsets (in rotations, CCW+, -0.5 to 0.5 range)
  public static final double FL_CANCODER_OFFSET = 0.3884 * -1.0;
  public static final double FR_CANCODER_OFFSET = 0.21167 * -1.0;
  public static final double RL_CANCODER_OFFSET = -0.0049 * -1.0;
  public static final double RR_CANCODER_OFFSET = -0.1782 * -1.0;

  public static final double CANCODER_RANGE = 0.5;
  public static final SensorDirectionValue CANCODER_DIRECTION = 
    SensorDirectionValue.CounterClockwise_Positive;

  // Drive Kraken X60
  public static final double WHEEL_CIRCUMFERENCE = Units.inchesToMeters(4.0) * Math.PI;
  public static final double DRIVE_RATIO = 6.75; // SDS Mk4i L2

  //TODO
  private static final Slot0Configs DRIVE_MOTOR_GAINS = new Slot0Configs()
    .withKP(0.96573).withKI(0.0).withKD(0.0)
    .withKS(0.088062).withKV(0.77811).withKA(0.013299);

  private static final CurrentLimitsConfigs DRIVE_MOTOR_CURRENT = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(45).withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

  private static final FeedbackConfigs DRIVE_ENCODER = new FeedbackConfigs()
    .withSensorToMechanismRatio(DRIVE_RATIO);
    //.withSensorToMechanismRatio(1.0);

  private static final AudioConfigs DRIVE_MOTOR_AUDIO = new AudioConfigs()
    .withBeepOnBoot(false).withBeepOnConfig(true);

  public static final TalonFXConfiguration DRIVE_CONFIG = new TalonFXConfiguration()
    .withSlot0(DRIVE_MOTOR_GAINS)
    .withCurrentLimits(DRIVE_MOTOR_CURRENT)
    .withFeedback(DRIVE_ENCODER)
    .withAudio(DRIVE_MOTOR_AUDIO);

  // Steer NEO
  public static final double STEER_RATIO = 150.0 / 7.0;

  public static final SparkBaseConfig STEER_CONFIG = new SparkMaxConfig()
    .inverted(true)
    .smartCurrentLimit(20)
    .idleMode(IdleMode.kBrake)
    .apply(new ClosedLoopConfig()
      .pidf(0.45, 0.00001, 0.0, 0.0)
      .positionWrappingInputRange(-0.5 * STEER_RATIO, 0.5 * STEER_RATIO)
      .positionWrappingEnabled(true)
    );

  // ADIS16470 Gyro
  public static final IMUAxis GYRO_YAW = IMUAxis.kZ;
  public static final IMUAxis GYRO_PITCH = IMUAxis.kX;
  public static final IMUAxis GYRO_ROLL = IMUAxis.kY;

  // Base size
  public static final Translation2d BASE_DIMENSIONS = 
    new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(25));

  // Module positions
  /*
   * Positive x values represent moving toward the front of the robot whereas positive y values 
   * represent moving toward the left of the robot.
   */
  private static final double EDGE = Units.inchesToMeters(2.625); // Distance edge of module to center of wheel

  public static final Translation2d[] POSITIONS = {
    new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // FL
    new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE), // FR
    new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // RL
    new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE)  // RR
  };

  // Auto config
  public static final PPHolonomicDriveController AUTO_CONTROLLER = 
    new PPHolonomicDriveController(
      new PIDConstants(2.25, 0.0, 0.0), // Translational
      new PIDConstants(0.0, 0.0, 0.0) // Rotational
    );
}
