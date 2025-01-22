package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

  /** Prevent instantiation */
  private Util() {}
}