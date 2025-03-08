package frc.robot.util;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;

public class AlignmentVector {
  /** Pre-defined CENTER alignment vectors */
  public static final AlignmentVector[] CENTER_VECTORS = {
    new AlignmentVector("REEF_1", new Translation2d(3.66, 4.02), Rotation2d.fromDegrees(0)),
    new AlignmentVector("REEF_2", new Translation2d(4.07, 3.3), Rotation2d.fromDegrees(60)),
    new AlignmentVector("REEF_3", new Translation2d(4.9, 3.3), Rotation2d.fromDegrees(120)),
    new AlignmentVector("REEF_4", new Translation2d(5.32, 4.02), Rotation2d.fromDegrees(180)),
    new AlignmentVector("REEF_5", new Translation2d(4.9, 4.74), Rotation2d.fromDegrees(240)),
    new AlignmentVector("REEF_6", new Translation2d(4.07, 4.74), Rotation2d.fromDegrees(300))
  };

  /** Pre-defined LEFT alignment vectors */
  public static final AlignmentVector[] LEFT_VECTORS = {
    new AlignmentVector("REEF_A", new Translation2d(3.66, 4.184), Rotation2d.fromDegrees(0)),
    new AlignmentVector("REEF_C", new Translation2d(3.928, 3.382), Rotation2d.fromDegrees(60)),
    new AlignmentVector("REEF_E", new Translation2d(4.758, 3.218), Rotation2d.fromDegrees(120)),
    new AlignmentVector("REEF_G", new Translation2d(5.32, 3.856), Rotation2d.fromDegrees(180)),
    new AlignmentVector("REEF_I", new Translation2d(5.042, 4.658), Rotation2d.fromDegrees(240)),
    new AlignmentVector("REEF_K", new Translation2d(4.212, 4.822), Rotation2d.fromDegrees(300))
  };

  /** Pre-defined RIGHT alignment vectors */
  public static final AlignmentVector[] RIGHT_VECTORS = {
    new AlignmentVector("REEF_B", new Translation2d(3.66, 3.856), Rotation2d.fromDegrees(0)),
    new AlignmentVector("REEF_D", new Translation2d(4.212, 3.218), Rotation2d.fromDegrees(60)),
    new AlignmentVector("REEF_F", new Translation2d(5.042, 3.382), Rotation2d.fromDegrees(120)),
    new AlignmentVector("REEF_H", new Translation2d(5.32, 4.184), Rotation2d.fromDegrees(180)),
    new AlignmentVector("REEF_J", new Translation2d(4.758, 4.822), Rotation2d.fromDegrees(240)),
    new AlignmentVector("REEF_L", new Translation2d(3.928, 4.658), Rotation2d.fromDegrees(300))
  };

  public final String name;
  public final Translation2d target, unitDirection;
  public final Rotation2d direction;
  public final Vector<N2> unitDirectionVector;

  /**
   * Helper class for storing alignment targets.
   * @param name
   * @param target Target point to reach.
   * @param direction Direction of vector to drive along, originating from target.
   */
  public AlignmentVector(String name, Translation2d target, Rotation2d direction) {
    this.name = name;
    this.target = target;
    this.direction = direction;
    unitDirection = new Translation2d(1.0, direction);
    unitDirectionVector = unitDirection.toVector();
  }

  /**
   * Helper class for storing alignment targets.
   * @param name
   * @param target Target point to reach.
   * @param direction The rotation between this point and target defines the direction of the vector.
   */
  public AlignmentVector(String name, Translation2d target, Translation2d lookAt) {
    this(name, target, lookAt.minus(target).getAngle());
  }

  @Override
  public String toString() {
    return String.format("AlignmentVector(%s)", name);
  }
}
