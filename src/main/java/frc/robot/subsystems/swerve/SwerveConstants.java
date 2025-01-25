package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.FlippingUtil;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.Constants.BuildConstants;

/**
 * Constants specific to the SwerveDrive base
 */
public final class SwerveConstants {
  // Speed configs
  public static final double MAX_WHEEL_VELOCITY = 5.0; // Absolute max wheel m/s

  // Max drive speeds
  public static final double FAST_TRANSLATIONAL_SPEED = 5.0; // m/s
  public static final double FAST_ROTATIONAL_SPEED = 12.438; // rad/s

  public static final double SLOW_TRANSLATIONAL_SPEED = FAST_TRANSLATIONAL_SPEED * 0.5;
  public static final double SLOW_ROTATIONAL_SPEED = FAST_TRANSLATIONAL_SPEED * 0.5;

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
      .pidf(0.45, 0.00001, 0.0, 0.0, ClosedLoopSlot.kSlot0)
      .positionWrappingInputRange(-0.5 * STEER_RATIO, 0.5 * STEER_RATIO)
      .positionWrappingEnabled(true)
    );

  // ADIS16470 Gyro
  public static final IMUAxis GYRO_YAW = IMUAxis.kZ;
  public static final IMUAxis GYRO_PITCH = IMUAxis.kX;
  public static final IMUAxis GYRO_ROLL = IMUAxis.kY;

  // Odometry
  public static final Matrix<N3, N1> STATE_STD_DEVS = VecBuilder.fill(0.3, 0.3, 0.1);

  // Base size
  public static final Translation2d BASE_DIMENSIONS = 
    new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(25));

  // Module positions
  /*
   * Positive x values represent moving toward the front of the robot whereas positive y values 
   * represent moving toward the left of the robot.
   */
  private static final double EDGE = Units.inchesToMeters(2.625); // Distance edge of module to center of wheel
  public static final double BUMPER_WIDTH = Units.inchesToMeters(0.75 + 2.5);

  public static final Translation2d[] POSITIONS = {
    new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // FL
    new Translation2d(BASE_DIMENSIONS.getX() / 2.0 - EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE), // FR
    new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / 2.0 - EDGE), // RL
    new Translation2d(BASE_DIMENSIONS.getX() / -2.0 + EDGE, BASE_DIMENSIONS.getY() / -2.0 + EDGE)  // RR
  };

  // Auto config
  public static final PPHolonomicDriveController AUTO_CONTROLLER = 
    new PPHolonomicDriveController(
      new PIDConstants(19.303, 0.0, 1.3841), // Translational
      new PIDConstants(2.0, 0.0, 0.0) // Rotational
    );

  // Default positions, if none is set by the auto program by PREGAME
  public static final Pose2d DEFAULT_BLUE_POSE = BuildConstants.ON_OFFICIAL_FIELD ?
    // Actual field, against the reef
    new Pose2d(3.3 - (BASE_DIMENSIONS.getX() / 2.0) - BUMPER_WIDTH, 4.026, Rotation2d.kZero) :
    // ITC map default pose, 1 square from the origin
    new Pose2d(0.6096, 0.6096, Rotation2d.kZero);
  
  public static final Pose2d DEFAULT_RED_POSE = FlippingUtil.flipFieldPose(DEFAULT_BLUE_POSE);
}
