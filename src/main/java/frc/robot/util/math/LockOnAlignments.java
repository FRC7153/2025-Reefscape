package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class LockOnAlignments {
  // MARK: Reef Scoring

  /** Alignment vectors for REEF LEFT scoring */
  public static final AlignmentVector[] REEF_LEFT_VECTORS = {
    new AlignmentVector("REEF_A", new Translation2d(3.658, 4.249), Rotation2d.fromDegrees(0), 18, 7),
new AlignmentVector("REEF_C", new Translation2d(3.931, 3.447), Rotation2d.fromDegrees(60), 17, 8),
new AlignmentVector("REEF_E", new Translation2d(4.762, 3.282), Rotation2d.fromDegrees(120), 22, 9),
new AlignmentVector("REEF_G", new Translation2d(5.321, 3.92), Rotation2d.fromDegrees(180), 21, 10),
new AlignmentVector("REEF_I", new Translation2d(5.047, 4.722), Rotation2d.fromDegrees(240), 20, 11),
new AlignmentVector("REEF_K", new Translation2d(4.216, 4.886), Rotation2d.fromDegrees(300), 19, 6)
  };

  /** Alignment vectors for REEF CENTER scoring */
  public static final AlignmentVector[] REEF_CENTER_VECTORS = {
    new AlignmentVector("REEF_1", new Translation2d(3.658, 4.084), Rotation2d.fromDegrees(0), 18, 7),
new AlignmentVector("REEF_2", new Translation2d(4.074, 3.365), Rotation2d.fromDegrees(60), 17, 8),
new AlignmentVector("REEF_3", new Translation2d(4.905, 3.365), Rotation2d.fromDegrees(120), 22, 9),
new AlignmentVector("REEF_4", new Translation2d(5.321, 4.084), Rotation2d.fromDegrees(180), 21, 10),
new AlignmentVector("REEF_5", new Translation2d(4.905, 4.804), Rotation2d.fromDegrees(240), 20, 11),
new AlignmentVector("REEF_6", new Translation2d(4.074, 4.804), Rotation2d.fromDegrees(300), 19, 6)
  };

  /** Alignment vectors for REEF RIGHT scoring */
  public static final AlignmentVector[] REEF_RIGHT_VECTORS = {
    new AlignmentVector("REEF_B", new Translation2d(3.658, 3.92), Rotation2d.fromDegrees(0), 18, 7),
new AlignmentVector("REEF_D", new Translation2d(4.216, 3.282), Rotation2d.fromDegrees(60), 17, 8),
new AlignmentVector("REEF_F", new Translation2d(5.047, 3.447), Rotation2d.fromDegrees(120), 22, 9),
new AlignmentVector("REEF_H", new Translation2d(5.321, 4.249), Rotation2d.fromDegrees(180), 21, 10),
new AlignmentVector("REEF_J", new Translation2d(4.762, 4.886), Rotation2d.fromDegrees(240), 20, 11),
new AlignmentVector("REEF_L", new Translation2d(3.931, 4.722), Rotation2d.fromDegrees(300), 19, 6)
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
}
