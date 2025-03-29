package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopDriveCommand extends Command {
  private final SwerveDrive drive;
  private final Supplier<Double> ySupplier, xSupplier, thetaSupplier;
  private final BooleanSupplier fastMode, robotOrientedSupplier, rearRobotOrientedSupplier;

  /**
   * @param drive
   * @param ySupplier Left is +, percentage
   * @param xSupplier Forward is +, percentage
   * @param thetaSupplier CCW+, percentage
   * @param fastMode
   * @param robotOrientedSupplier If true, drives robot oriented. If false, drives field oriented.
   * @param rearRobotOrientedSupplier If true, drives robot-rear oriented.
   */
  public TeleopDriveCommand(
    SwerveDrive drive, 
    Supplier<Double> ySupplier, 
    Supplier<Double> xSupplier, 
    Supplier<Double> thetaSupplier,
    BooleanSupplier fastMode,
    BooleanSupplier robotOrientedSupplier,
    BooleanSupplier rearRobotOrientedSupplier
  ) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    this.xSupplier = xSupplier;
    this.thetaSupplier = thetaSupplier;
    this.fastMode = fastMode;
    this.robotOrientedSupplier = robotOrientedSupplier;
    this.rearRobotOrientedSupplier = rearRobotOrientedSupplier;

    addRequirements(drive);
  }

  @Override
  public void execute() {
    // Do not do anything if not in teleop or test
    if (!DriverStation.isTeleopEnabled() && !DriverStation.isTestEnabled()) {
      drive.stop();
      return;
    }

    double x = xSupplier.get();
    double y = ySupplier.get();
    double theta = thetaSupplier.get();

    // Apply deadbands
    x = Math.abs(x) > 0.075 ? x : 0.0;
    y = Math.abs(y) > 0.075 ? y : 0.0;
    theta = Math.abs(theta) > 0.075 ? theta : 0.0;

    // Flip, if rear robot oriented
    boolean rearOriented = rearRobotOrientedSupplier.getAsBoolean();

    if (rearOriented) {
      x = -x;
      y = -y;
      theta = theta * 0.75; // decrease rotation while rear oriented
    }

    drive.drive(
      y * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
      x * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
      theta * (fastMode.getAsBoolean() ? SwerveConstants.FAST_ROTATIONAL_SPEED : SwerveConstants.SLOW_ROTATIONAL_SPEED), 
      false, 
      !(robotOrientedSupplier.getAsBoolean() || rearOriented)
    );
  }

  @Override
  public void end(boolean terminated) {
    drive.stop();
  }
}
