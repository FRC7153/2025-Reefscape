package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
    return (value < min) ? 0 : value;
  }

  /** Prevent instantiation */
  private Util() {}
}