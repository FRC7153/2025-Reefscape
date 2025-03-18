package frc.robot.commands.alignment;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.math.AlignmentVector;

/**
 * Extend this class to lock on to different AligmentVectors.
 */
public class AutoLockOnCommand extends Command {
  // Empty double array, for resetting Limelight tag filters
  private static final double[] EMPTY_TAG_SET = new double[0];

  // Instance members
  private final SwerveDrive drive;
  private final AlignmentVector vector;
  private final double velocity, threshold, timeout;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  private double projectionScalar;
  private double mostRecentDistance;
  private double startTime;

  /**
   * Locks onto the supplied AlignmentVector.
   * @param drive
   * @param vector Vector to follow;
   * @param velocity Velocity along vector, m/s
   * @param threshold Meters from target before this command is terminated.
   * @param timeout Max number of seconds this command can run
   */
  public AutoLockOnCommand(
    SwerveDrive drive, 
    AlignmentVector vector,
    double velocity,
    double threshold,
    double timeout
  ) {
    this.drive = drive;
    this.vector = vector;
    this.velocity = velocity;
    this.threshold = threshold;
    this.timeout = timeout;

    // Init target state
    targetState.heading = vector.getDirection();
    targetState.linearVelocity = velocity;

    addRequirements(drive);
  }

  // MARK: Target initialization
  @Override
  public void initialize() {
    // Init projection
    Pose2d currentPose = drive.getPosition(false);
    projectionScalar = vector.getPointProjectionScalar(currentPose.getTranslation());

    mostRecentDistance = currentPose.getTranslation().getDistance(vector.getTarget());
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
    
    // Determine target position
    projectionScalar += (velocity * TimedRobot.kDefaultPeriod); // Position offset = requested velocity * time
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
  }

  // MARK: End
  @Override
  public void end(boolean interrupted) {
    drive.setLimelightTagFilter(EMPTY_TAG_SET);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return mostRecentDistance <= threshold || (Timer.getFPGATimestamp() - startTime) >= timeout;
  }

  @Override
  public String getName() {
    return String.format("AutoLockOnCommand(%s)", vector.getName());
  }
}
