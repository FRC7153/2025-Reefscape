package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Alignment vectors used for automating lining up with various targets. */
public class LockOnAlignments {
  // MARK: Reef Scoring

  /** (Robot-oriented) alignment vectors for REEF LEFT scoring */
  public static final AlignmentVector[] RO_REEF_LEFT_VECTORS = {
    new AlignmentVector("REEF_A", new Translation2d(3.658, 4.249), Rotation2d.fromDegrees(0), 18, 7),
    new AlignmentVector("REEF_C", new Translation2d(3.876, 3.415), Rotation2d.fromDegrees(60), 17, 8),
    new AlignmentVector("REEF_E", new Translation2d(4.707, 3.187), Rotation2d.fromDegrees(120), 22, 9),
    new AlignmentVector("REEF_G", new Translation2d(5.321, 3.793), Rotation2d.fromDegrees(180), 21, 10),
    new AlignmentVector("REEF_I", new Translation2d(5.102, 4.626), Rotation2d.fromDegrees(240), 20, 11),
    new AlignmentVector("REEF_K", new Translation2d(4.271, 4.854), Rotation2d.fromDegrees(300), 19, 6)
  };

  /** Alignment vectors for REEF CENTER scoring */
  public static final AlignmentVector[] REEF_CENTER_VECTORS = {
    new AlignmentVector("REEF_1", new Translation2d(3.658, 4.084), Rotation2d.fromDegrees(0), 18, 7),
    new AlignmentVector("REEF_2", new Translation2d(4.019, 3.333), Rotation2d.fromDegrees(60), 17, 8),
    new AlignmentVector("REEF_3", new Translation2d(4.85, 3.269), Rotation2d.fromDegrees(120), 22, 9),
    new AlignmentVector("REEF_4", new Translation2d(5.321, 3.957), Rotation2d.fromDegrees(180), 21, 10),
    new AlignmentVector("REEF_5", new Translation2d(4.96, 4.709), Rotation2d.fromDegrees(240), 20, 11),
    new AlignmentVector("REEF_6", new Translation2d(4.129, 4.772), Rotation2d.fromDegrees(300), 19, 6)
  };

  /** (Robot-oriented) Alignment vectors for REEF RIGHT scoring */
  public static final AlignmentVector[] RO_REEF_RIGHT_VECTORS = {
    new AlignmentVector("REEF_B", new Translation2d(3.658, 3.92), Rotation2d.fromDegrees(0), 18, 7),
    new AlignmentVector("REEF_D", new Translation2d(4.161, 3.251), Rotation2d.fromDegrees(60), 17, 8),
    new AlignmentVector("REEF_F", new Translation2d(4.992, 3.352), Rotation2d.fromDegrees(120), 22, 9),
    new AlignmentVector("REEF_H", new Translation2d(5.321, 4.122), Rotation2d.fromDegrees(180), 21, 10),
    new AlignmentVector("REEF_J", new Translation2d(4.817, 4.791), Rotation2d.fromDegrees(240), 20, 11),
    new AlignmentVector("REEF_L", new Translation2d(3.986, 4.69), Rotation2d.fromDegrees(300), 19, 6)
  };

  /** Alignment vectors for REEF RIGHT (driver station-oriented) scoring */
  public static final AlignmentVector[] REEF_LEFT_VECTORS = {
    RO_REEF_LEFT_VECTORS[0], RO_REEF_LEFT_VECTORS[1], RO_REEF_RIGHT_VECTORS[2], RO_REEF_RIGHT_VECTORS[3], 
    RO_REEF_RIGHT_VECTORS[4], RO_REEF_LEFT_VECTORS[5]
  };

  /** Alignment vectors for REEF RIGHT (driver station-oriented) scoring */
  public static final AlignmentVector[] REEF_RIGHT_VECTORS = {
    RO_REEF_RIGHT_VECTORS[0], RO_REEF_RIGHT_VECTORS[1], RO_REEF_LEFT_VECTORS[2], RO_REEF_LEFT_VECTORS[3],
    RO_REEF_LEFT_VECTORS[4], RO_REEF_RIGHT_VECTORS[5]
  };

  /** Center of reef */
  public static final Translation2d REEF_CENTER = new Translation2d(4.476, 4.493);

  /** Each corner of the reef extended outwards a certain amount (used to define triangular regions below) */
  private static final Translation2d[] REEF_CORNERS_EXPANDED = {
    new Translation2d(-8.514, 11.993),
    new Translation2d(-8.514, -3.007),
    new Translation2d(4.476, -10.507),
    new Translation2d(17.467, -3.007),
    new Translation2d(17.467, 11.993),
    new Translation2d(4.476, 19.493)
  };

  /** Each reef zone, in the standard ordering */
  public static final Triangle2d[] REEF_ZONES = {
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[0], REEF_CORNERS_EXPANDED[1]), // A/B
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[1], REEF_CORNERS_EXPANDED[2]), // C/D
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[2], REEF_CORNERS_EXPANDED[3]), // E/F
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[3], REEF_CORNERS_EXPANDED[4]), // G/H
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[4], REEF_CORNERS_EXPANDED[5]), // I/J
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[5], REEF_CORNERS_EXPANDED[0]), // K/L
  };

  // MARK: Coral Stations
  private static final Rotation2d LEFT_CORAL_STATION = Rotation2d.fromDegrees(126);
  private static final Rotation2d RIGHT_CORAL_STATION = Rotation2d.fromDegrees(-126);

  /**
   * Determines if the robot should lock on to the left coral station, right coral station, or cages.
   * @param allianceRelativePosition The alliance-relative position of the robot on the field.
   * @return The rotation of the nearest coral station or cages.
   */
  public static Rotation2d getBestRotationTarget(Translation2d allianceRelativePosition) {
    if (allianceRelativePosition.getX() > REEF_CENTER.getX()) {
      // Lock onto cages
      return Rotation2d.k180deg;
    } else {
      // Lock onto a coral station
      return
        (allianceRelativePosition.getY() >= REEF_CENTER.getY()) ? LEFT_CORAL_STATION : RIGHT_CORAL_STATION;
    }
  }

  /** Utility class, prevent instantiation. */
  private LockOnAlignments() {}
}
