package frc.robot;

import com.ctre.phoenix6.CANBus;

public final class Constants {
  public static final class BuildConstants {
    public static final boolean PUBLISH_EVERYTHING = true;
  }
  
  public static final class HardwareConstants {
    public static final int PDH_CAN = 1;

    // Swerve Drive hardware
    public static final int FL_DRIVE_KRAKEN_CAN = 2;
    public static final int FL_STEER_NEO_CAN = 3;
    public static final int FL_STEER_CANCODER_CAN = 4;
    public static final int FR_DRIVE_KRAKEN_CAN = 5;
    public static final int FR_STEER_NEO_CAN = 6;
    public static final int FR_STEER_CANCODER_CAN = 7;
    public static final int RL_DRIVE_KRAKEN_CAN = 8;
    public static final int RL_STEER_NEO_CAN = 9;
    public static final int RL_STEER_CANCODER_CAN = 10;
    public static final int RR_DRIVE_KRAKEN_CAN = 11;
    public static final int RR_STEER_NEO_CAN = 12;
    public static final int RR_STEER_CANCODER_CAN = 13;

    public static final CANBus CANIVORE = new CANBus("CANivore");
  }
}