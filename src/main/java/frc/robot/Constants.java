package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.Elevator.ElevatorState;

public final class Constants {
  public static final class BuildConstants {
    public static final double EPSILON = 1E-6;

    public static final boolean PUBLISH_EVERYTHING = true;
    public static final boolean INCLUDE_TEST_AUTOS = true;

    /**
     * If true, the real field layout is used and limelight pipeline 1 is used.
     * If false, the ITC layout is used and limelight pipeline 0 is used.
     * Also effects default positions
     */
    public static final boolean ON_OFFICIAL_FIELD = false;
    public static final AprilTagFields DEFAULT_FIELD = AprilTagFields.k2025ReefscapeAndyMark; // See TU-12
  }

  public static final class DashboardConstants {
    public static final int ELASTIC_SERVER_PORT = 5800;
    public static final String ELASTIC_DRIVE_TAB = "Drive";
  }

  public static final class ClimberConstants {
    public static final double CLIMBER_RATIO = 20.0;

    public static final SparkBaseConfig CLIMBER_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kCoast)
      .inverted(true) 
      .smartCurrentLimit(80);
  }

  public static final class ManipulatorConstants {
    public static final double MANIPULATOR_RATIO = 1.0;
     
    private static final AbsoluteEncoderConfig MANIPULATOR_ABSOLUTE_ENCODER_CONFIG = new AbsoluteEncoderConfig()
      .zeroOffset(ElevatorConstants.MANIPULATOR_PIVOT_OFFSET)
      .inverted(true)
      .zeroCentered(true);
    
    private static final LimitSwitchConfig MANIPULATOR_LIMIT_SWITCH_CONFIG = new LimitSwitchConfig()
      .forwardLimitSwitchEnabled(false)
      .reverseLimitSwitchEnabled(false)
      .forwardLimitSwitchType(Type.kNormallyOpen);

    public static final SparkBaseConfig MANIPULATOR_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(30)
      .apply(MANIPULATOR_ABSOLUTE_ENCODER_CONFIG)
      .apply(MANIPULATOR_LIMIT_SWITCH_CONFIG);  
  }

  public static final class ElevatorConstants {
    public static final double ELEVATOR_RATIO = 7.75;
    public static final double MANIPULATOR_PIVOT_RATIO = 25.0; // used to be 5.0
    public static final double MANIPULATOR_PIVOT_OFFSET = 0.776622;

    /*private static final Slot0Configs ELEVATOR_MOTOR_GAINS = new Slot0Configs()
      .withKP(6.9615).withKI(0.0).withKD(1.0762)
      .withKS(0.0).withKV(1.414).withKA(0.97985)
      .withKG(0.71188).withGravityType(GravityTypeValue.Elevator_Static);*/

    private static final Slot0Configs ELEVATOR_MOTOR_GAINS = new Slot0Configs()
      .withKP(3.596).withKI(0.0).withKD(0.0)
      .withKS(0.1233).withKV(1.3441).withKA(0.0)
      .withKG(0.70347).withGravityType(GravityTypeValue.Elevator_Static);
    
    //TODO
    private static final Slot0Configs MANIPULATOR_PIVOT_GAINS = new Slot0Configs()
    .withKP(0.0).withKI(0.0).withKD(0.0)
    .withKS(0.0).withKV(0.0).withKA(0.0)
    .withKG(0.0).withGravityType(GravityTypeValue.Arm_Cosine);

    private static final CurrentLimitsConfigs ELEVATOR_MOTOR_CURRENT = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

    private static final CurrentLimitsConfigs MANIPULATOR_PIVOT_CURRENT = new CurrentLimitsConfigs()
      .withSupplyCurrentLimit(60).withSupplyCurrentLimitEnable(true)
      .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs ELEVATOR_ENCODER = new FeedbackConfigs()
      .withSensorToMechanismRatio(ELEVATOR_RATIO)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    private static final MotorOutputConfigs ELEVATOR_OUTPUT = new MotorOutputConfigs()
      .withInverted(InvertedValue.Clockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

    private static final MotorOutputConfigs MANIPULATOR_PIVOT_OUTPUT = new MotorOutputConfigs()
      .withInverted(InvertedValue.CounterClockwise_Positive)
      .withNeutralMode(NeutralModeValue.Brake);

    private static final FeedbackConfigs MANIPULATOR_PIVOT_ENCODER = new FeedbackConfigs()
      .withSensorToMechanismRatio(MANIPULATOR_PIVOT_RATIO)
      .withFeedbackSensorSource(FeedbackSensorSourceValue.RotorSensor);

    public static final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration()
      .withSlot0(ELEVATOR_MOTOR_GAINS)
      .withCurrentLimits(ELEVATOR_MOTOR_CURRENT)
      .withFeedback(ELEVATOR_ENCODER)
      .withAudio(HardwareConstants.TALON_AUDIO_CONFIG)
      .withMotorOutput(ELEVATOR_OUTPUT);
      
    //TODO find if inverted, config PIDF
    public static final TalonFXConfiguration MANIPULATOR_PIVOT_CONFIG = new TalonFXConfiguration()
      .withSlot0(MANIPULATOR_PIVOT_GAINS)
      .withCurrentLimits(MANIPULATOR_PIVOT_CURRENT)
      .withFeedback(MANIPULATOR_PIVOT_ENCODER)
      .withAudio(HardwareConstants.TALON_AUDIO_CONFIG)
      .withMotorOutput(MANIPULATOR_PIVOT_OUTPUT);
  }

  public static final class ElevatorPositions {
    public static final ElevatorState STOW = new ElevatorState(0.0, 0.0);
    public static final ElevatorState INTAKE = new ElevatorState(0.0, 0.0);
    public static final ElevatorState PROCESSOR = new ElevatorState(0.0, 0.0);

    public static final ElevatorState L1 = new ElevatorState(0.0, 0.0);
    public static final ElevatorState L2 = new ElevatorState(0.0, 0.0);
    public static final ElevatorState L3 = new ElevatorState(0.0, 0.0);
    public static final ElevatorState L4 = new ElevatorState(0.0, 0.0);

    public static final ElevatorState ALGAE_LOW = new ElevatorState(0.0, 0.0);
    public static final ElevatorState ALGAE_HIGH = new ElevatorState(0.0, 0.0);
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
    public static final int CLIMBER_CAN = 18;

    // Manipulator Hardware 
    public static final int MANIPULATOR_CAN = 19;

    // CAN Busses
    public static final CANBus RIO_CAN = new CANBus("rio");
    public static final CANBus CANIVORE = new CANBus("CANivore");

    // CTRE Specific
    public static final AudioConfigs TALON_AUDIO_CONFIG = new AudioConfigs()
      .withBeepOnBoot(false).withBeepOnConfig(true);
  }
}
