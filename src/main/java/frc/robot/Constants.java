package frc.robot;

import com.ctre.phoenix6.CANBus;

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
    public static final double kCLIMBER_RATIO = 0.0;

    public static final int kCLIMBER_CURRENT_LIMIT = 40;

    //TODO
    public static final double kCLIMBER_P = 0.0;
    public static final double kCLIMBER_I = 0.0;
    public static final double kCLIMBER_D = 0.0;
  }

  public static final class ManipulatorConstants {
    //TODO
    public static final double kManipulator_RATIO = 0.0;

    public static final int kMANIPULATOR_CURRENT_LIMIT = 30;

    //TODO
    public static final double kMANIPULATOR_P = 0.0;
    public static final double kMANIPULATOR_I = 0.0;
    public static final double kMANIPULATOR_D = 0.0;
  }

  public static final class ElevatorConstants {
    //TODO
    public static final double kELEVATOR_RATIO = 0.0;
    public static final double kMANIPULATOR_PIVOT_RATIO = 0.0;

    public static final double kMANIPULATOR_PIVOT_CURRENT_LIMIT = 40;
    public static final double kELEVATOR_CURRENT_LIMIT = 0.0;

    //TODO 
    public static final double kELEVATOR_P = 0.0;
    public static final double kELEVATOR_I = 0.0;
    public static final double kELEVATOR_D = 0.0;
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
    public static final int ELEVATOR_RIGHT_CAN = 14;
    public static final int ELEVATOR_LEFT_CAN = 15;
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