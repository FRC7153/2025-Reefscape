package frc.robot.commands.alignment;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.math.AlignmentVector;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

/**
 * Extend this class to lock on to different AligmentVectors.
 */
public class AutoLockOnCommand extends Command {
  // Empty double array, for resetting Limelight tag filters
  private static final double[] EMPTY_TAG_SET = new double[0];

  // Instance members
  private final SwerveDrive drive;
  private final AlignmentVector vector;
  private final double velocity, translationThreshold, rotationThreshold, timeout;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  private double projectionScalar;
  private double mostRecentDistance;
  private double mostRecentAngleDiff;
  private double startTime;

  /**
   * Locks onto the AlignmentVector.
   * @param drive
   * @param vector Vector to follow;
   * @param velocity Velocity along vector, m/s
   * @param threshold Meters from target before this command is terminated.
   * @param rotationThreshold Degrees from target before this command is terminated.
   * @param timeout Max number of seconds this command can run
   */
  public AutoLockOnCommand(
    SwerveDrive drive, 
    AlignmentVector vector,
    double velocity,
    double translationThreshold,
    double rotationThreshold,
    double timeout
  ) {
    this.drive = drive;
    this.vector = vector;
    this.velocity = velocity;
    this.translationThreshold = translationThreshold;
    this.rotationThreshold = rotationThreshold;
    this.timeout = timeout;

    // Init target state
    targetState.heading = vector.getDirection();

    addRequirements(drive);
  }

  /**
   * Locks onto the AlignmentVector for a certain amount of time.
   * @param drive
   * @param vector Vector to follow
   * @param velocity Velocity, along vector, m/s
   * @param forTime Amount of seconds to run for.
   */
  public AutoLockOnCommand(
    SwerveDrive drive,
    AlignmentVector vector,
    double velocity,
    double forTime
  ) {
    this(drive, vector, velocity, -1.0, -1.0, forTime);
  }

  // MARK: Target initialization
  @Override
  public void initialize() {
    // Set limelight throttle
    drive.setLimelightThrottle(LimelightConstants.TARGETING_THROTTLE);

    // Init projection
    Pose2d currentPose = drive.getPosition(false);
    projectionScalar = vector.getPointProjectionScalar(currentPose.getTranslation());

    mostRecentDistance = currentPose.getTranslation().getDistance(vector.getTarget());
    mostRecentAngleDiff = Util.getAngleDifferenceDegrees(vector.getDirection(), currentPose.getRotation());
    startTime = Timer.getFPGATimestamp();

    // Update Limelight filters
    drive.setLimelightTagFilter(vector.getAprilTags());

    // Output
    System.out.printf("Now (auto) locking on to %s\n", vector.getName());
  }

  // MARK: Alignment
  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = drive.getPosition(false);
    
    // Determine target position, only if not within target
    if (mostRecentDistance > translationThreshold) {
      projectionScalar += (velocity * TimedRobot.kDefaultPeriod); // Position offset = requested velocity * time
      targetState.linearVelocity = velocity;
    } else {
      targetState.linearVelocity = 0;
    }

    Translation2d projection = vector.getPointOnVectorFromScalar(projectionScalar);

    // Get drive base speed
    targetState.pose = new Pose2d(projection, vector.getDirection());

    ChassisSpeeds chassis = 
      SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    Util.deadbandChassisSpeeds(chassis, 0.05, 0.01);
    
    // Drive
    drive.drive(chassis, true);

    // Update distance
    mostRecentDistance = currentPose.getTranslation().getDistance(vector.getTarget());
    mostRecentAngleDiff = Util.getAngleDifferenceDegrees(vector.getDirection(), currentPose.getRotation());
  }

  // MARK: End
  @Override
  public void end(boolean interrupted) {
    drive.setLimelightTagFilter(EMPTY_TAG_SET);
    drive.stop();

    // Set throttle
    drive.setLimelightThrottle(DriverStation.isEnabled() ? LimelightConstants.ENABLED_THROTTLE : LimelightConstants.DISABLED_THROTTLE);
  }

  @Override
  public boolean isFinished() {
    if (mostRecentDistance <= translationThreshold && mostRecentAngleDiff <= rotationThreshold) {
      // Has reached translation and rotation target thresholds
      return true;
    } else if ((Timer.getFPGATimestamp() - startTime) >= timeout) {
      // Has timed out
      if (translationThreshold != -1.0 && rotationThreshold != -1.0) {
        // Alert if it hasn't reached thresholds
        Elastic.sendNotification(
          new Notification(
            NotificationLevel.INFO, 
            "AutoLockOnCommand expired", 
            String.format("Timed out %.3f meters and %.3f degrees from target", mostRecentDistance, mostRecentAngleDiff), 
            1200
          )
        );
      }
      
      return true;
    }

    // Has not reached any threshold
    return false;
  }

  @Override
  public String getName() {
    return String.format("AutoLockOnCommand(%s)", vector.getName());
  }
}
