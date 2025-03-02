package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BuildConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.AlignmentVector;

/** aka "LockedInCommand" */
public class LockOnCommand extends Command {
  private static final double TARGET_WINDOW = 50; // Max +/- degrees to target

  // Shared output
  private static final DoublePublisher LockOnDistPublisher = 
    NetworkTableInstance.getDefault().getTable("Swerve").getDoubleTopic("LockOnDistance").publish();
  private static final StringPublisher LockOnTargetPublisher = 
    NetworkTableInstance.getDefault().getTable("Swerve").getStringTopic("LockOnTarget").publish();

  static {
    LockOnDistPublisher.set(0.0);
    LockOnTargetPublisher.set("N/A");
  }

  private final SwerveDrive drive;
  private final DoubleSupplier speedSupplier;
  private final BiConsumer<RumbleType, Double> rumbleConsumer;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
  private final AlignmentVector[] vectors;

  private AlignmentVector alignment;

  /**
   * Drives the robot along the specified vector, at a certain speed.
   * @param drive
   * @param speedSupplier Velocity along the supplier (%)
   * @param rumbleConsumer Rumble consumer for controller feedback when the robot hits target.
   * @param vectors Vectors to search through.
   */
  public LockOnCommand(
    SwerveDrive drive, 
    DoubleSupplier speedSupplier, 
    BiConsumer<RumbleType, Double> rumbleConsumer, 
    AlignmentVector[] vectors
  ) {
    this.drive = drive;
    this.speedSupplier = speedSupplier;
    this.rumbleConsumer = rumbleConsumer;
    this.vectors = vectors;

    if (vectors.length == 0) {
      throw new IllegalArgumentException("LockOnCommand needs at least one vector!");
    }

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    if (vectors.length == 1) {
      // Only one target!
      alignment = vectors[0];
    } else {
      // Find the best target vector by finding the closest target within window
      Pose2d currentPose = drive.getPosition(false);

      double bestDistance = vectors[0].target.getDistance(currentPose.getTranslation());
      alignment = vectors[0];

      for (int v = 1; v < vectors.length; v++) {
        // Get angle difference
        double angleDiff = (vectors[v].direction.getDegrees() - currentPose.getRotation().getDegrees()) % 360.0;

        if (angleDiff <= TARGET_WINDOW || angleDiff >= (360.0 - TARGET_WINDOW)) {
          // Within window, check distance
          double dist = vectors[v].target.getDistance(currentPose.getTranslation());
          if (dist < bestDistance) {
            bestDistance = dist;
            alignment = vectors[v];
          }
        }
      }
    }

    // Output
    LockOnTargetPublisher.set(alignment.name);
    System.out.printf("Now locking on to %s\n", alignment.name);
  }

  @Override
  public void execute() {
    // Determine target projection on vector
    Pose2d currentPose = drive.getPosition(false);
    Translation2d delta = currentPose.getTranslation().minus(alignment.target);

    double projectionScalar = delta.toVector().dot(alignment.unitDirectionVector);
    Translation2d projection = alignment.target.plus(alignment.unitDirection.times(projectionScalar));

    // Calculate velocity to projection
    targetState.pose = new Pose2d(projection, alignment.direction);
    ChassisSpeeds speeds = SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    // Add user input
    double userSpeed = speedSupplier.getAsDouble() * SwerveConstants.FAST_TRANSLATIONAL_SPEED;
    Rotation2d userInputToRobot = alignment.direction.minus(currentPose.getRotation());

    speeds.vxMetersPerSecond += userInputToRobot.getCos() * userSpeed;
    speeds.vyMetersPerSecond += userInputToRobot.getSin() * userSpeed;

    // Drive
    drive.drive(speeds, true);

    // Give user feedback
    double dist = currentPose.getTranslation().getDistance(alignment.target);

    double rumble = (Math.abs(dist) <= BuildConstants.EPSILON) ? 1.0 : Math.min(0.03 / dist, 1.0);
    rumbleConsumer.accept(RumbleType.kRightRumble, rumble);

    LockOnDistPublisher.set(dist);
  }

  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(RumbleType.kRightRumble, 0.0);
    LockOnTargetPublisher.set("N/A");
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }
}
