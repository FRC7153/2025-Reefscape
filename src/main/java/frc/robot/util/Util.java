package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.BuildConstants;
import frc.robot.util.logging.ConsoleLogger;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

public class Util {
  /**
   * @return The current stack trace, as a String.
   */
  public static String getCurrentStackTrace() {
    StringBuilder trace = new StringBuilder();

    for (StackTraceElement s : Thread.currentThread().getStackTrace()) {
      trace.append(s.toString());
      trace.append(", ");
    }

    return trace.toString();
  }

  /**
   * @return If the current alliance is red alliance
   */
  public static boolean isRedAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      // Attempted to check Alliance before it was received!
      String trace = getCurrentStackTrace();
      ConsoleLogger.reportError("Invalid alliance received in " + trace);
      
      Elastic.sendNotification(new Notification(
        NotificationLevel.WARNING,
        "Invalid alliance received from FMS",
        trace
      ));
    } else if (alliance.get().equals(Alliance.Red)) {
      // Red alliance confirmed
      return true;
    }

    // Either blue alliance or default
    return false;
  }

  /**
   * @param value Input value.
   * @param min All inputs less than this are rounded down to 0.
   * @return 0 if value is less than min, else value
   */
  public static double applyDeadband(double value, double min) {
    return (Math.abs(value) < min) ? 0 : value;
  }

  /**
   * @param tag Tag id
   * @return The 2d position of the tag on the field
   */
  public static Translation2d getTagPose(int tag) {
    Optional<Pose3d> pose = BuildConstants.FIELD.getTagPose(tag);

    if (pose.isEmpty()) {
      // Unknown tag
      ConsoleLogger.reportWarning(String.format("Unknown tag request (id %d)", tag));
      return Translation2d.kZero;
    } else {
      return new Translation2d(pose.get().getX(), pose.get().getY());
    }
  }

  /** Prevent instantiation */
  private Util() {}
}