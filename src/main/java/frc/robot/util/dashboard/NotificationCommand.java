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
   * @param level The level of the Notification.
   */
  public NotificationCommand(String title, String message, NotificationLevel level) {
    super(() -> {
      Elastic.sendNotification(new Notification(level, title, message, 150000));

      // Log
      switch (level) {
        case ERROR -> ConsoleLogger.reportError(title + ": " + message);
        case INFO -> System.out.println(title + ": " + message);
        case WARNING -> ConsoleLogger.reportWarning(title + ": " + message);
      }
    });
  }
}
