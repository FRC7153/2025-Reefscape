package frc.robot.commands;

import java.util.function.BiConsumer;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

/** aka "LockedInCommand" */
public class LockOnCommand extends Command {
  private final SwerveDrive drive;
  private final DoubleSupplier speedSupplier;
  private final BiConsumer<RumbleType, Double> rumbleConsumer;
  private final Translation2d target, unitDirection;
  private final Rotation2d direction;
  private final Vector<N2> unitDirectionVector;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  /**
   * Drives the robot along the specified vector, at a certain speed.
   * @param drive
   * @param speedSupplier Velocity along the supplier (%)
   * @param rumbleConsumer Rumble consumer for controller feedback when the robot hits target.
   * @param target Target point to reach.
   * @param lookAt Direction of vector to drive along, originating from target.
   */
  public LockOnCommand(
    SwerveDrive drive, 
    DoubleSupplier speedSupplier, 
    BiConsumer<RumbleType, Double> rumbleConsumer, 
    Translation2d target, 
    Rotation2d lookAt
  ) {
    this.drive = drive;
    this.speedSupplier = speedSupplier;
    this.rumbleConsumer = rumbleConsumer;
    this.target = target;
    this.direction = lookAt;
    unitDirection = new Translation2d(1.0, lookAt);
    unitDirectionVector = unitDirection.toVector();

    addRequirements(drive);
  }

  /**
   * Drives the robot along the specified vector, at a certain speed.
   * @param drive
   * @param speedSupplier Velocity along the supplier (%)
   * @param rumbleConsumer Rumble consumer for controller feedback when the robot hits target.
   * @param target Target point to reach.
   * @param lookAt The rotation between this point and target defines the direction of the vector.
   */
  public LockOnCommand(
    SwerveDrive drive, 
    DoubleSupplier speedSupplier, 
    BiConsumer<RumbleType, Double> rumbleConsumer,
    Translation2d target, 
    Translation2d lookAt
  ) {
    this(drive, speedSupplier, rumbleConsumer, target, lookAt.minus(target).getAngle());
  }

  @Override
  public void execute() {
    // Determine target projection on vector
    Pose2d currentPose = drive.getPosition(false);
    Translation2d delta = currentPose.getTranslation().minus(target);

    double projectionScalar = delta.toVector().dot(unitDirectionVector);
    Translation2d projection = target.plus(unitDirection.times(projectionScalar));

    // Drive to projection with user input
    targetState.pose = new Pose2d(projection, direction);
    ChassisSpeeds speeds = SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    //speeds.vxMetersPerSecond += speedSupplier.getAsDouble() * SwerveConstants.FAST_TRANSLATIONAL_SPEED;

    drive.drive(speeds, true);

    // Vibrate controller feedback
    double dist = currentPose.getTranslation().getDistance(target);
    rumbleConsumer.accept(RumbleType.kRightRumble, Math.min(0.03 / dist, 1.0)); // Adjust 0.03 to make more or less responsive
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }

  @Override
  public String getName() {
    return String.format("LockOnCommand(%s)", target);
  }
}
