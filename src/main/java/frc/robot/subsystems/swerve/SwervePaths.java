package frc.robot.subsystems.swerve;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

/**
 * Utility class for making Paths.
 */
public class SwervePaths {
  /**
   * @param drive
   * @param pathName Name of path to follow.
   * @param resetPosition Whether the position should be reset before following the path (for first
   * path in an auto).
   * @return Command to follow the path.
   */
  @SuppressWarnings("UseSpecificCatch")
  public static Command getFollowPathCommand(SwerveDrive drive, String pathName, boolean resetPosition) {
    String name = String.format("FollowPathCommand(%s, reset: %b)", pathName, resetPosition);

    try {
      // Load path and alliance color
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      
      if (Util.isRedAlliance()) {
        // Red alliance, need to flip path
        path = path.flipPath();
      }
      
      // Create path follow command
      Command followCommand = new FollowPathCommand(
        path, 
        drive.odometry::getFieldRelativePosition, 
        drive::getCurrentChassisSpeeds, 
        (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
          drive.drive(speeds, true);
        }, 
        SwerveConstants.AUTO_CONTROLLER, 
        drive.autoConfig, 
        () -> false, 
        drive
      );

      if (resetPosition) {
        // Reset position before running the path
        Pose2d origin = path.getStartingHolonomicPose().get();
        return new InstantCommand(() -> drive.resetOdometry(origin)).andThen(followCommand).withName(name);
      } else {
        // Just run the path
        return followCommand.withName(name);
      }
    } catch (Exception e) {
      // Failed to load path file
      ConsoleLogger.reportError(String.format("Failed to load path '%s': %s", pathName, e.getMessage()));
      Elastic.sendNotification(new Notification(
        NotificationLevel.ERROR, 
        String.format("Failed to load path '%s'", pathName), 
        e.getMessage())
      );
      return new PrintCommand(String.format("Running failed path: '%s'", pathName)).withName(name);
    }
  }
  /** Prevent instantiation */
  private SwervePaths() {}
}
