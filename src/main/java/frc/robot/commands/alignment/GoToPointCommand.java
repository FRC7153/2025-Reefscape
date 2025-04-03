package frc.robot.commands.alignment;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.math.AlignmentVector;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

public class GoToPointCommand extends Command {
  private final SwerveDrive drive;
  private final Pose2d target;
  private final double translationThreshold, rotationThreshold, timeout;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  private double startTime, mostRecentDistance, mostRecentAngleDiff;

  public GoToPointCommand(SwerveDrive drive, Pose2d target, double translationThreshold, double rotationThreshold, double timeout) {
    this.drive = drive;
    this.target = target;
    this.translationThreshold = translationThreshold;
    this.rotationThreshold = rotationThreshold;
    this.timeout = timeout;

    targetState.pose = target;
    targetState.heading = target.getRotation();

    addRequirements(drive);
  }

  public GoToPointCommand(SwerveDrive drive, AlignmentVector target, double translationThreshold, double rotationThreshold, double timeout) {
    this(drive, new Pose2d(target.getTarget(), target.getDirection()), translationThreshold, rotationThreshold, timeout);
  }

  @Override
  public void initialize() {
    Pose2d currentPose = drive.getPosition(false);

    startTime = Timer.getFPGATimestamp();
    mostRecentDistance = currentPose.getTranslation().getDistance(target.getTranslation());
    mostRecentAngleDiff = Util.getAngleDifferenceDegrees(currentPose.getRotation(), target.getRotation());
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPosition(false);

    ChassisSpeeds chassis = 
      SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    Util.deadbandChassisSpeeds(chassis, 0.05, 0.01);
    
    // Drive
    drive.drive(chassis, true);

    // Update distances
    mostRecentDistance = currentPose.getTranslation().getDistance(target.getTranslation());
    mostRecentAngleDiff = Util.getAngleDifferenceDegrees(currentPose.getRotation(), target.getRotation());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
  
  @Override
  public boolean isFinished() {
    if (mostRecentDistance <= translationThreshold && mostRecentAngleDiff <= rotationThreshold) {
      // Within thresholds
      return true;
    } else if (Timer.getFPGATimestamp() - startTime >= timeout) {
      // Timed out
      Elastic.sendNotification(
        new Notification(
          NotificationLevel.INFO, 
          "AutoLockOnCommand expired", 
          String.format("Timed out %.3f meters and %.3f degrees from target", mostRecentDistance, mostRecentAngleDiff), 
          1200
        )
      );

      return true;
    }

    return false;
  }
}
