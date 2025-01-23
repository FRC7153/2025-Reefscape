package frc.robot;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
public final class Constants {
  public static final class BuildConstants {
    public static final boolean PUBLISH_EVERYTHING = true;
    public static final boolean INCLUDE_TEST_AUTOS = true;

    /**
     * If true, the real field layout is used and limelight pipeline 1 is used.
     * If false, the ITC layout is used and limelight pipeline 0 is used.
     * Also effects default positions
     */
    public static final boolean ON_OFFICIAL_FIELD = false;
  }

  public static final class DashboardConstants {
    public static final int ELASTIC_SERVER_PORT = 5800;
    public static final String ELASTIC_DRIVE_TAB = "Drive";
  }

  public static final class ClimberConstants {
    //TODO
    public static final double kCLIMBER_RATIO = 20.0;

    public static final int kCLIMBER_CURRENT_LIMIT = 40;

    //TODO find if inverted, config PIDF
    public static final SparkBaseConfig CLIMBER_CONFIG = new SparkFlexConfig()
    .idleMode(IdleMode.kBrake)
    .inverted(false)
    .smartCurrentLimit(40)
    .apply(new ClosedLoopConfig()
      .pidf(0.0, 0.0, 0.0, 0.0,  ClosedLoopSlot.kSlot0));
  }

  public static final class ManipulatorConstants {
    //TODO
    public static final double kManipulator_RATIO = 1.0;

    public static final int kMANIPULATOR_CURRENT_LIMIT = 30;

    //TODO
    public static final double kMANIPULATOR_P = 0.0;
    public static final double kMANIPULATOR_I = 0.0;
    public static final double kMANIPULATOR_D = 0.0;
  }

  public static final class ElevatorConstants {
    //TODO
    public static final double kELEVATOR_RATIO = 7.75;
    public static final double kMANIPULATOR_PIVOT_RATIO = 4.0;

    public static final int kMANIPULATOR_PIVOT_CURRENT_LIMIT = 40;
    public static final int kELEVATOR_CURRENT_LIMIT = 40;

    //TODO 
    public static final double kELEVATOR_P = 0.0;
    public static final double kELEVATOR_I = 0.0;
    public static final double kELEVATOR_D = 0.0;

    //TODO
    private static final Slot0Configs ELEVATOR_MOTOR_GAINS = new Slot0Configs()
    .withKP(0.0).withKI(0.0).withKD(0.0)
    .withKS(0.0).withKV(0.0).withKA(0.0);

    private static final CurrentLimitsConfigs ELEVATOR_MOTOR_CURRENT = new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(50).withSupplyCurrentLimitEnable(true)
    .withStatorCurrentLimit(80).withStatorCurrentLimitEnable(true);

    private static final FeedbackConfigs ELEVATOR_ENCODER = new FeedbackConfigs()
    .withSensorToMechanismRatio(kELEVATOR_RATIO);

    private static final AudioConfigs ELEVATOR_MOTOR_AUDIO = new AudioConfigs()
    .withBeepOnBoot(false).withBeepOnConfig(true);

    public static final TalonFXConfiguration ELEVATOR_CONFIG = new TalonFXConfiguration()
      .withSlot0(ELEVATOR_MOTOR_GAINS)
      .withCurrentLimits(ELEVATOR_MOTOR_CURRENT)
      .withFeedback(ELEVATOR_ENCODER)
      .withAudio(ELEVATOR_MOTOR_AUDIO);
      
    //TODO find if inverted, config PIDF
    public static final SparkBaseConfig MANIPULATOR_PIVOT_CONFIG = new SparkFlexConfig()
      .idleMode(IdleMode.kBrake)
      .inverted(false)
      .smartCurrentLimit(40)
      .apply(new ClosedLoopConfig()
        .pidf(0.0, 0.0, 0.0, 0.0));
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
    public static final int ELEVATOR_MAIN_CAN = 14;
    public static final int ELEVATOR_FOLLOWER_CAN = 15;
    public static final int MANIPULATOR_PIVOT_CAN = 16;

    // Climber Hardware
    public static final int CLIMBER_RIGHT_CAN = 17;
    public static final int CLIMBER_LEFT_CAN = 18;

    // Manipulator Hardware 
    public static final int MANIPULATOR_CAN = 19;

    public static final CANBus RIO_CAN = new CANBus("rio");
    public static final CANBus CANIVORE = new CANBus("CANivore");
  }
}
