package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.Elevator.ElevatorState;

public final class Constants {
  public static final class BuildConstants {
    public static final double EPSILON = 1E-6;

    public static final boolean PUBLISH_EVERYTHING = true;
    public static final boolean INCLUDE_TEST_AUTOS = true;

    public static final AprilTagFieldLayout FIELD = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark); // See TU-12
  }

  public static final class DashboardConstants {
    public static final int ELASTIC_SERVER_PORT = 5800;
    public static final String ELASTIC_DRIVE_TAB = "Drive";
  }

  public static final class ClimberConstants {
    public static final double CLIMBER_RATIO = 125.0; 

    public static final ClosedLoopConfig CLIMBER_PIVOT_MOTOR_GAINS = new ClosedLoopConfig()
      .pid(0.0, 0.0, 0.0, ClosedLoopSlot.kSlot0)
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    public static final SparkBaseConfig CLIMBER_PIVOT_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .smartCurrentLimit(60)
      .apply(CLIMBER_PIVOT_MOTOR_GAINS);

    public static final SparkBaseConfig CLIMBER_WINCH_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(true)
      .smartCurrentLimit(80);
  }

  public static final class ManipulatorConstants {
    public static final double MANIPULATOR_RATIO = 1.0;

    public static final SparkBaseConfig MANIPULATOR_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(60);
  }

  public static final class ElevatorConstants {
    public static final double ELEVATOR_RATIO = 7.75;
    public static final double MANIPULATOR_PIVOT_RATIO = 25.0;
    public static final double MANIPULATOR_PIVOT_OFFSET = (0.776622 + .25) % 1.0;
    public static final double MANIPULATOR_PIVOT_DEFAULT_POS = 0.397217;

    private static final Slot0Configs ELEVATOR_MOTOR_GAINS = new Slot0Configs()
      .withKP(3.596).withKI(0.0).withKD(0.0)
      .withKS(0.1103).withKV(0.92196).withKA(0.0019)
      .withKG(0.5609).withGravityType(GravityTypeValue.Elevator_Static);

    private static final MotionMagicConfigs ELEVATOR_MM_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(3.0) // 4.5
      .withMotionMagicAcceleration(5.5); // 9.5

    private static final Slot0Configs MANIPULATOR_PIVOT_GAINS = new Slot0Configs()
      .withKP(57.811).withKI(0.0).withKD(6.2107)
      .withKS(0.1085).withKV(2.854).withKA(0.0022)
      .withKG(0.2945).withGravityType(GravityTypeValue.Arm_Cosine);

    private static final MotionMagicConfigs MANIPULATOR_MM_CONFIGS = new MotionMagicConfigs()
      .withMotionMagicCruiseVelocity(5.5)
      .withMotionMagicAcceleration(2.0);

    private static final CurrentLimitsConfigs ELEVATOR_MOTOR_CURRENT = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(70).withStatorCurrentLimitEnable(true);

    private static final CurrentLimitsConfigs MANIPULATOR_PIVOT_CURRENT = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs ELEVATOR_ENCODER = new FeedbackConfigs()
      .withSensorToMechanismRatio(ELEVATOR_RATIO)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    private static final MotorOutputConfigs ELEVATOR_OUTPUT = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

    private static final MotorOutputConfigs MANIPULATOR_PIVOT_OUTPUT = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

    private static final FeedbackConfigs MANIPULATOR_PIVOT_ENCODER = new FeedbackConfigs()
      .withSensorToMechanismRatio(MANIPULATOR_PIVOT_RATIO)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration()
      .withSlot0(ELEVATOR_MOTOR_GAINS)
      .withMotionMagic(ELEVATOR_MM_CONFIGS)
      .withCurrentLimits(ELEVATOR_MOTOR_CURRENT)
      .withFeedback(ELEVATOR_ENCODER)
      .withAudio(HardwareConstants.TALON_AUDIO_CONFIG)
      .withMotorOutput(ELEVATOR_OUTPUT);
      
    public static final TalonFXConfiguration MANIPULATOR_PIVOT_CONFIG = new TalonFXConfiguration()
      .withSlot0(MANIPULATOR_PIVOT_GAINS)
      .withMotionMagic(MANIPULATOR_MM_CONFIGS)
      .withCurrentLimits(MANIPULATOR_PIVOT_CURRENT)
      .withFeedback(MANIPULATOR_PIVOT_ENCODER)
      .withAudio(HardwareConstants.TALON_AUDIO_CONFIG)
      .withMotorOutput(MANIPULATOR_PIVOT_OUTPUT);

    // Sensor Spark Max configs
    private static final AbsoluteEncoderConfig MANIPULATOR_ABSOLUTE_ENCODER_CONFIG = new AbsoluteEncoderConfig()
      .zeroOffset(ElevatorConstants.MANIPULATOR_PIVOT_OFFSET)
      .inverted(true)
      .zeroCentered(true);

    private static final LimitSwitchConfig ALGAE_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig()
      .forwardLimitSwitchEnabled(false)
      .reverseLimitSwitchEnabled(false)
      .forwardLimitSwitchType(Type.kNormallyOpen);

    public static final SparkBaseConfig MANIPULATOR_SENSOR_CONFIG = new SparkFlexConfig()
      .apply(MANIPULATOR_ABSOLUTE_ENCODER_CONFIG)
      .apply(ALGAE_LIMIT_SWITCH_CONFIG);
  }

  public static final class ElevatorPositions {
    public static final ElevatorState STOW = new ElevatorState(0.15, 0.4);
    public static final ElevatorState INTAKE = new ElevatorState(1.0, 0.3);
    public static final ElevatorState PROCESSOR = new ElevatorState(0.1, 0.2);

    public static final ElevatorState L1 = new ElevatorState(0.05, 0.255);
    public static final ElevatorState L2 = new ElevatorState(1.4, 0.11); 
    public static final ElevatorState L3 = new ElevatorState(2.6, 0.06);
    public static final ElevatorState L4 = new ElevatorState(4.45, -0.02);

    public static final ElevatorState ALGAE_LOW = new ElevatorState(0.55, 0.2);
    public static final ElevatorState ALGAE_HIGH = new ElevatorState(1.55, 0.2);
  }

  public static final class LEDColors {
    public static final double RED = 0.61;
    public static final double BLUE = 0.87;
    public static final double GREEN = 0.71;
    public static final double YELLOW = 0.69;
    public static final double BLACK = 0.99;
  }

  public static final class HardwareConstants {
    public static final int PDH_CAN = 1;

    // Swerve Drive Hardware
    public static final int FR_DRIVE_KRAKEN_CAN = 2;
    public static final int FR_STEER_NEO_CAN = 3;
    public static final int FR_STEER_CANCODER_CAN = 4;
    public static final int RR_DRIVE_KRAKEN_CAN = 5;
    public static final int RR_STEER_NEO_CAN = 6;
    public static final int RR_STEER_CANCODER_CAN = 7;
    public static final int RL_DRIVE_KRAKEN_CAN = 8;
    public static final int RL_STEER_NEO_CAN = 9;
    public static final int RL_STEER_CANCODER_CAN = 10;
    public static final int FL_DRIVE_KRAKEN_CAN = 11;
    public static final int FL_STEER_NEO_CAN = 12;
    public static final int FL_STEER_CANCODER_CAN = 13;

    // Elevator Hardware
    public static final int ELEVATOR_LEADER_CAN = 14;
    public static final int ELEVATOR_FOLLOWER_CAN = 15;
    public static final int MANIPULATOR_PIVOT_CAN = 16;

    // Climber Hardware
    public static final int CLIMBER_WINCH = 18;
    public static final int CLIMBER_PIVOT = 17;

    // Manipulator Hardware 
    public static final int MANIPULATOR_CAN = 19;
    public static final int MANIPULATOR_SENSORS_CAN = 20; // Spark Max, no motor

    // CAN Busses
    public static final CANBus RIO_CAN = new CANBus("rio");
    public static final CANBus CANIVORE = new CANBus("CANivore");

    // CTRE Specific
    public static final AudioConfigs TALON_AUDIO_CONFIG = new AudioConfigs()
      .withBeepOnBoot(false).withBeepOnConfig(true);

    // LED
    public static final int LED_PWM_PORT = 9;
  }

  public static final class LEDConstants {
    public static double LED_COLOR = 0.0; 
  }
}
