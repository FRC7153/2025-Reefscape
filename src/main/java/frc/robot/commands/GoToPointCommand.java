package frc.robot.commands;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;

public class GoToPointCommand extends Command {
  private final SwerveDrive drive;
  private final PathPlannerTrajectoryState targetState;
  
  public GoToPointCommand(SwerveDrive drive, Pose2d target) {
    this.drive = drive;

    targetState = new PathPlannerTrajectoryState();
    targetState.pose = target;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    if (Util.isRedAlliance()) {
      targetState.pose = FlippingUtil.flipFieldPose(targetState.pose);
    }
  }

  @Override
  public void execute() {
    Pose2d currentPose = drive.odometry.getFieldRelativePosition();
    ChassisSpeeds speeds = SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    // 
    //if (speeds.vxMetersPerSecond < 0.05 && speeds.vyMetersPerSecond < 0.05 && speeds.omegaRadiansPerSecond < )
    System.out.printf("x:%f y:%f theta:%f \n", speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);

    drive.drive(speeds, true);
  }

  @Override
  public void end(boolean  interrupted) {
    if (!interrupted) {
      drive.stop();
    }
  }
}
