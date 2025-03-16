package frc.robot.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BuildConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;

public class GoToPointCommand extends Command {
  // Logging
  private static StructPublisher<Pose2d> targetPub;

  private final SwerveDrive drive;

  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();
  private final Pose2d target;
  
  /**
   * @param drive
   * @param target Target point (alliance relative!)
   */
  public GoToPointCommand(SwerveDrive drive, Pose2d target) {
    this.drive = drive;
    this.target = target;

    addRequirements(drive);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      if (targetPub == null) {
        NetworkTable nt = NetworkTableInstance.getDefault().getTable("GoToPointSetpoint");
        targetPub = nt.getStructTopic("Target", Pose2d.struct).publish();
      }
    }
  }

  @Override
  public void initialize() {
    if (Util.isRedAlliance()) {
      targetState.pose = FlippingUtil.flipFieldPose(target);
    } else {
      targetState.pose = target;
    }

    if (BuildConstants.PUBLISH_EVERYTHING) {
      targetPub.set(targetState.pose);
    }
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.getPosition(true);
    ChassisSpeeds speeds = SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    // Apply deadbands
    Util.deadbandChassisSpeeds(speeds, 0.05, 0.01);

    drive.drive(speeds, true);
  }

  @Override
  public void end(boolean  interrupted) {
    drive.stop();
  }

  @Override
  public String getName() {
    return String.format("GoToPointCommand(%s)", target);
  }
}
