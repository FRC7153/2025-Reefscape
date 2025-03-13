package frc.robot.util.math;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.BuildConstants;

public class AlignmentVector {
  private final String name;
  private final Translation2d target;
  private final Translation2d unitDirection;
  private final Rotation2d direction;
  private final Vector<N2> unitDirectionVector;

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

    // Publish this alignment vector to NT for verification
    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("AlignmentVectors");
      StructPublisher<Pose2d> pub = nt.getStructTopic(name, Pose2d.struct).publish();
      pub.set(new Pose2d(target, direction));
    }
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

  /**
   * Projects a 2d position onto this vector, with a specified offset.
   * @param point The point to project.
   * @param scalarOffset A distance added to the vector scalar.
   * @return The projected point.
   */
  public Translation2d projectPoint(Translation2d point, double scalarOffset) {
    Translation2d delta = point.minus(target);

    double scalar = delta.toVector().dot(unitDirectionVector);
    scalar += scalarOffset;
    return target.plus(unitDirection.times(scalar));
  }

  /**
   * Projects a 2d position onto this vector.
   * @param input The point to project.
   * @return The projected point.
   */
  public Translation2d projectPoint(Translation2d input) {
    return projectPoint(input, 0.0);
  }

  /**
   * Projects a vector onto this vector.
   * @param vector The vector to project.
   * @return The projected vector.
   */
  public Translation2d projectVector(Translation2d vector) {
    double scalar = vector.toVector().dot(unitDirectionVector);
    // Unit direction vector magnitude is always 1, so we can ignore it

    return vector.times(scalar);
  }

  public Translation2d getTarget() {
    return target;
  }

  public Rotation2d getDirection() {
    return direction;
  }

  public String getName() {
    return name;
  }

  @Override
  public String toString() {
    return String.format("AlignmentVector(%s)", name);
  }
}
