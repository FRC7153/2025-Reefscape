package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.BuildConstants;
import frc.robot.commands.AsyncDeferredCommand;
import frc.robot.commands.GoToPointCommand;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

/**
 * Utility class for making Paths.
 */
public class SwervePaths {
  private static final PathConstraints UnlimitedConstraints = PathConstraints.unlimitedConstraints(12);

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

  public static Command getGoToPointCommand(SwerveDrive drive, Pose2d target) {
    return new AsyncDeferredCommand(
      String.format("GoToPointGeneratorCommand(%s)", target.toString()), 
      () -> {
        // Get waypoints
        Pose2d start = drive.getPosition(true);
        Pose2d end = Util.isRedAlliance() ? FlippingUtil.flipFieldPose(target) : target;

        Pose2d inter = start.interpolate(end, 0.5);

        // Get direction of travel
        Translation2d diff = end.getTranslation().minus(inter.getTranslation());

        if (Math.abs(diff.getX()) < BuildConstants.EPSILON && Math.abs(diff.getY()) < BuildConstants.EPSILON) {
          // Rotation2d components will be zero
          return new PrintCommand("[GoToPointCommand] Diff too close to 0 to generate trajectory.");
        }

        Rotation2d directionOfTravel = new Rotation2d(diff.getX(), diff.getY());

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
          new Pose2d(inter.getX(), inter.getY(), directionOfTravel),
          new Pose2d(end.getX(), end.getY(), directionOfTravel)
        );

        // Create path
        PathPlannerPath path = new PathPlannerPath(
          waypoints, 
          UnlimitedConstraints, 
          null, 
          new GoalEndState(0.0, end.getRotation())
        );

        // Return command
        return new FollowPathCommand(
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
        ).withName(String.format("GoToPointCommand(from: %s, to: %s)", start, end));
      }, 
      drive
    ).andThen(new GoToPointCommand(drive, target));
  }

  /** Prevent instantiation */
  private SwervePaths() {}
}
