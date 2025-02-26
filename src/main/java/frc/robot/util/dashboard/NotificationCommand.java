package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.util.logging.ConsoleLogger;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

public class NotificationCommand extends InstantCommand {
  /**
   * Command that sends the dashboard a notification and logs it.
   * @param title Title of notification.
   * @param message Message of notification.
   * @param error If true, the notification is an 'error'. If false, the notification is a 
   * 'warning'.
   */
  public NotificationCommand(String title, String message, boolean error) {
    super(() -> {
      Elastic.sendNotification(
        new Notification(
          error ? NotificationLevel.ERROR : NotificationLevel.WARNING, 
          title, 
          message,
          150000
        )
      );

      if (error) {
        ConsoleLogger.reportError(title + ": " + message);
      } else {
        ConsoleLogger.reportWarning(title + ": " + message);
      }
    });
  }
}
