package frc.robot;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Translation2d;

public final class Constants {
  public static final class BuildConstants {
    public static final boolean PUBLISH_EVERYTHING = true;
    public static final boolean INCLUDE_TEST_AUTOS = true;

    public static final AprilTagFieldLayout APRIL_TAG_LAYOUT = 
      AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  }
  

  public static final class LimelightConstants {
    // TODO
    public static final double limelightMountingAngle = 0.0; // Limelight Mounting Angle in degrees
    // TODO
    public static final double correctionAngle = Math.atan(Math.toRadians(limelightMountingAngle)); 
  }
  public static final class HardwareConstants {
    public static final int PDH_CAN = 1;

    // Swerve Drive hardware
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

    public static final CANBus RIO_CAN = new CANBus("rio");
    public static final CANBus CANIVORE = new CANBus("CANivore");
  }

  public static final class FieldConstants {
    /** Array containing each AprilTag position */
    public static final Translation2d[] APRIL_TAG_POSITIONS = {
      Translation2d.kZero, // There is no tag id 0
      new Translation2d(16.697198, 0.65532),
      new Translation2d(16.697198, 7.3964799999999995),
      new Translation2d(11.560809999999998, 8.05561),
      new Translation2d(9.276079999999999, 6.137656),
      new Translation2d(9.276079999999999, 1.914906),
      new Translation2d(13.474446, 3.3063179999999996),
      new Translation2d(13.890498, 4.0259),
      new Translation2d(13.474446, 4.745482),
      new Translation2d(12.643358, 4.745482),
      new Translation2d(12.227305999999999, 4.0259),
      new Translation2d(12.643358, 3.3063179999999996),
      new Translation2d(0.851154, 0.65532),
      new Translation2d(0.851154, 7.3964799999999995),
      new Translation2d(8.272272, 6.137656),
      new Translation2d(8.272272, 1.914906),
      new Translation2d(5.9875419999999995, -0.0038099999999999996),
      new Translation2d(4.073905999999999, 3.3063179999999996),
      new Translation2d(3.6576, 4.0259),
      new Translation2d(4.073905999999999, 4.745482),
      new Translation2d(4.904739999999999, 4.745482),
      new Translation2d(5.321046, 4.0259),
      new Translation2d(4.904739999999999, 3.3063179999999996)
    };
  }
}