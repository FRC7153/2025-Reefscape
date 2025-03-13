package frc.robot.util.dashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.BuildConstants;

/**
 * Utility class for telemetry for "LockOn..." (alignment) commands.
 */
public final class LockOnTelemetry {
  public static enum TargetType { REEF, CAGE, CORAL_STATION, ALGAE_SCORING }

  // Shared lock on output
  private static final DoublePublisher distPub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getDoubleTopic("Distance").publish();
  private static final StringPublisher targetPub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStringTopic("TargetName").publish();
  private static final StructPublisher<Pose2d> setpointPub = BuildConstants.PUBLISH_EVERYTHING ?
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStructTopic("Setpoint", Pose2d.struct).publish() : 
    null;

  // Shared target output
  private static final StringPublisher targetTypePub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStringTopic("Type").publish();

  private static TargetType currentType;

  static {
    // Init target type
    setTargetType(TargetType.REEF);
  }

  /**
   * @param targetName The name of the target
   */
  public static void setTargetName(String targetName) {
    targetPub.set(targetName);
    System.out.printf("Now locking on to %s\n", targetName);
  }

  /**
   * Clears the target name from NT.
   */
  public static void clearTargetName() {
    targetPub.set("N/A");
  }

  /**
   * @param setpoint The setpoint (projection + scalar) to drive to.
   * @param distance The distance to the target.
   */
  public static void setTargetSetpoint(Pose2d setpoint, double distance) {
    distPub.set(distance);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      setpointPub.set(setpoint);
    }
  }

  /**
   * Sets the current type of target to use.
   * @param type The type of target.
   */
  public static void setTargetType(TargetType type) {
    currentType = type;
    targetTypePub.set(type.name());
    System.out.printf("Now locking on to targets of type %s\n", type.name());
  }

  /**
   * @return The current type of target.
   */
  public static TargetType getTargetType() {
    return currentType;
  }

  /** Prevent instantiation */
  private LockOnTelemetry() {}
}
