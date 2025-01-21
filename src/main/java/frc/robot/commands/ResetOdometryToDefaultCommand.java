package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;

/**
 * Resets the swerve drive odometry to a default position
 */
public class ResetOdometryToDefaultCommand extends InstantCommand {
  public ResetOdometryToDefaultCommand(SwerveDrive drive) {
    super(() -> {
      if (Util.isRedAlliance()) {
        // Red alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_RED_POSE);
      } else {
        // Blue alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_BLUE_POSE);
      }
    }, drive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
