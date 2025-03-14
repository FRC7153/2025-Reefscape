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

  private final double[] aprilTags;

  /**
   * Helper class for storing alignment targets.
   * @param name
   * @param target Target point to reach.
   * @param direction Direction of vector to drive along, originating from target.
   * @param aprilTags April tag IDs to use to target.
   */
  public AlignmentVector(String name, Translation2d target, Rotation2d direction, int... aprilTags) {
    this.name = name;
    this.target = target;
    this.direction = direction;
    unitDirection = new Translation2d(1.0, direction);
    unitDirectionVector = unitDirection.toVector();

    // Copy ints into a double array for publishing
    double[] aprilTagDoubleArray = new double[aprilTags.length];

    for (int i = 0; i < aprilTags.length; i++) {
      aprilTagDoubleArray[i] = aprilTags[i];
    }

    this.aprilTags = aprilTagDoubleArray;

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
   * Projects a vector onto this vector and returns the magnitude.
   * @param vector The vector to project.
   * @return The projected vector's magnitude (may be negative).
   */
  public double getProjectedVectorMagnitude(Translation2d vector) {
    // Unit direction vector magnitude is always 1, so we can ignore it
    return vector.toVector().dot(unitDirectionVector);
  }

  public Translation2d getTarget() {
    return target;
  }

  public Rotation2d getDirection() {
    return direction;
  }

  public double[] getAprilTags() {
    return aprilTags;
  }

  public String getName() {
    return name;
  }

  @Override
  public String toString() {
    return String.format("AlignmentVector(%s)", name);
  }
}
