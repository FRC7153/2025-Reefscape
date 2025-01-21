package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

public class Util {
  /**
   * @return If the current alliance is blue alliance
   */
  public static boolean isBlueAlliance() {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      // Attempted to check Alliance before it was received!
      ConsoleLogger.reportError("Invalid alliance received");
      Thread.dumpStack();
      
      Elastic.sendNotification(new Notification(
        NotificationLevel.WARNING,
        "Invalid alliance received from FMS",
        ""
      ));
    } else if (alliance.get().equals(Alliance.Red)) {
      // Red alliance confirmed
      return false;
    }

    return true;
  }
}