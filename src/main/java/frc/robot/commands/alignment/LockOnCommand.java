package frc.robot.commands.alignment;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BuildConstants;
import frc.robot.commands.alignment.LockOnTargetChooserCommand.TargetType;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;
import frc.robot.util.math.AlignmentVector;
import frc.robot.util.math.LockOnAlignments;

/**
 * Extend this class to lock on to different AligmentVectors.
 */
public class LockOnCommand extends Command {
  /** Represents which set of targets to use. */
  public static enum TargetGroup { LEFT, CENTER, RIGHT }

  // Empty double array, for resetting Limelight tag filters
  private static final double[] EMPTY_TAG_SET = new double[0];

  // Shared LockOn output
  private static final DoublePublisher distPub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getDoubleTopic("Distance").publish();
  private static final StringPublisher targetPub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStringTopic("TargetName").publish();
  private static final StructPublisher<Pose2d> setpointPub = BuildConstants.PUBLISH_EVERYTHING ?
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStructTopic("Setpoint", Pose2d.struct).publish() : 
    null;

  private final SwerveDrive drive;
  private final Supplier<Double> xSupplier, ySupplier;
  private final BiConsumer<RumbleType, Double> rumbleConsumer;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  private final TargetGroup group;
  private final AlignmentVector[] reefVectorGroup;

  private AlignmentVector vector;

  /**
   * Locks onto the supplied AlignmentVector.
   * @param drive
   * @param vectorSupplier Method that returns the best vector to lock on to.
   * @param xSupplier X input supplier (%)
   * @param ySupplier Y input supplier (%)
   * @param rumbleConsumers Consumers for haptic feedback
   */
  public LockOnCommand(
    SwerveDrive drive, 
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    BiConsumer<RumbleType, Double> rumbleConsumers,
    TargetGroup group
  ) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rumbleConsumer = rumbleConsumers;
    this.group = group;

    // Get vectors for the reef
    reefVectorGroup = switch (group) {
      case LEFT -> LockOnAlignments.REEF_LEFT_VECTORS;
      case RIGHT -> LockOnAlignments.REEF_RIGHT_VECTORS;
      case CENTER -> LockOnAlignments.REEF_CENTER_VECTORS;
    };

    addRequirements(drive);
  }

  // MARK: Target initialization
  @Override
  public void initialize() {
    // Determine which vector to use
    vector = null;
    TargetType type = LockOnTargetChooserCommand.getTargetType();

    if (type == TargetType.REEF) {
      // Scoring on reef
      // Determine which side of the reef the robot is on
      Pose2d currentPose = drive.getPosition(false);

      for (int i = 0; i < LockOnAlignments.REEF_ZONES.length; i++) {
        if (LockOnAlignments.REEF_ZONES[i].containsPoint(currentPose.getTranslation())) {
          // We are in this zone
          vector = reefVectorGroup[i];
          break;
        }
      }
      
      if (vector == null) {
        // We are in no zones?
        ConsoleLogger.reportError("Robot position is not in any reef zone!");
        vector = reefVectorGroup[0];
      }
    } else {
      // TODO alignment for cage, loading, and algae
      vector = reefVectorGroup[0];
    }

    // Update Limelight filters
    drive.setLimelightTagFilter(vector.getAprilTags());

    // Output
    targetPub.set(vector.getName());
    System.out.printf("Now locking on to %s\n", vector.getName());
  }

  // MARK: Alignment
  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = drive.getPosition(false);
    
    // Get user input (note y and x are swapped here, because forward (y+) should be a vector of 0 degrees)
    Translation2d speed = new Translation2d(
      ySupplier.get() * SwerveConstants.FAST_TRANSLATIONAL_SPEED,
      xSupplier.get() * SwerveConstants.FAST_TRANSLATIONAL_SPEED
    );

    double projectedSpeedMagnitude = vector.getProjectedVectorMagnitude(speed);

    System.out.printf("Speed is %.3f/%.3f\n", projectedSpeedMagnitude, SwerveConstants.FAST_TRANSLATIONAL_SPEED);

    // Determine target position
    Translation2d projection = vector.projectPoint(
      currentPose.getTranslation(), 
      projectedSpeedMagnitude * TimedRobot.kDefaultPeriod // Position offset = requested velocity * time
    );

    // Get drive base speed
    targetState.pose = new Pose2d(projection, vector.getDirection());
    targetState.heading = vector.getDirection();
    targetState.linearVelocity = projectedSpeedMagnitude;

    ChassisSpeeds chassis = 
      SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    Util.deadbandChassisSpeeds(chassis, 0.05, 0.01);
    
    // Drive
    drive.drive(chassis, true);

    // Feedback
    double dist = currentPose.getTranslation().getDistance(vector.getTarget());

    double rumble = (Math.abs(dist) <= BuildConstants.EPSILON) ? 1.0 : Math.min(0.007 / dist, 1.0);
    rumbleConsumer.accept(RumbleType.kRightRumble, rumble);

    distPub.set(dist);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      setpointPub.set(targetState.pose);
    }
  }

  // MARK: End
  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(RumbleType.kRightRumble, 0.0);
    targetPub.set("N/A");

    drive.setLimelightTagFilter(EMPTY_TAG_SET);

    if (interrupted) drive.stop();
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }

  @Override
  public String getName() {
    return String.format("LockOnCommand(%s)", group.name());
  }
}
