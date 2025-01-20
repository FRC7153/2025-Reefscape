package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.ConsoleLogger;

/**
 * Resets the swerve drive odometry to a default position
 */
public class ResetOdometryToDefaultCommand extends InstantCommand {
  public ResetOdometryToDefaultCommand(SwerveDrive drive) {
    super(() -> {
      Optional<Alliance> alliance = DriverStation.getAlliance();
      
      if (alliance.isEmpty()) {
        // No alliance color
        ConsoleLogger.reportError("ResetOdometryToDefaultCommand ran, but no alliance color!");
        drive.resetOdometry(SwerveConstants.DEFAULT_BLUE_POSE);
      } else if (alliance.get().equals(Alliance.Blue)) {
        // Blue alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_BLUE_POSE);
      } else {
        // Red alliance default pose
        drive.resetOdometry(SwerveConstants.DEFAULT_RED_POSE);
      }
    }, drive);
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
