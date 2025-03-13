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
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.math.AlignmentVector;

/**
 * Extend this class to lock on to different AligmentVectors.
 */
public class LockOnCommand extends Command {
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

  private final Supplier<AlignmentVector> vectorSupplier;
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
    Supplier<AlignmentVector> vectorSupplier,
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    BiConsumer<RumbleType, Double> rumbleConsumers
  ) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rumbleConsumer = rumbleConsumers;
    this.vectorSupplier = vectorSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Determine which side of the reef to use
    vector = vectorSupplier.get();

    // Output
    targetPub.set(vector.getName());
    System.out.printf("Now locking on to %s\n", vector.getName());
  }

  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = drive.getPosition(false);
    
    // Get user input (note y and x are swapped here, because forward (y+) should be a vector of 0 degrees)
    Translation2d speed = new Translation2d(
      ySupplier.get() * SwerveConstants.SLOW_TRANSLATIONAL_SPEED,
      xSupplier.get() * SwerveConstants.SLOW_TRANSLATIONAL_SPEED
    );

    double projectedSpeedMagnitude = vector.projectVector(speed).getNorm();

    // Determine target position
    Translation2d projection = vector.projectPoint(
      currentPose.getTranslation(), 
      projectedSpeedMagnitude * TimedRobot.kDefaultPeriod
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

  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(RumbleType.kRightRumble, 0.0);
    targetPub.set("N/A");

    if (interrupted) drive.stop();
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }
}
