package frc.robot.commands.alignment;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SnapRotationCommand extends Command {
  private final SwerveDrive drive;
  private final Supplier<Double> ySupplier, xSupplier;
  private final BooleanSupplier fastMode;
  private final Supplier<Rotation2d> targetSupplier;

  private Rotation2d target;

  /**
   * Snaps the rotation to a certain rotational target, while still allowing teleop driving.
   * @param drive
   * @param ySupplier Left is +, percentage
   * @param xSupplier Forward is +, percentage
   * @param fastMode
   * @param targetSupplier
   */
  public SnapRotationCommand(
    SwerveDrive drive, 
    Supplier<Double> ySupplier, 
    Supplier<Double> xSupplier, 
    BooleanSupplier fastMode,
    Supplier<Rotation2d> targetSupplier
  ) {
    this.drive = drive;
    this.ySupplier = ySupplier;
    this.xSupplier = xSupplier;
    this.fastMode = fastMode;
    this.targetSupplier = targetSupplier;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    target = targetSupplier.get();
    System.out.printf("Locking on to %.3f deg\n", target.getDegrees());
  }

  @Override
  public void execute() {
    Rotation2d current = drive.getPosition(false).getRotation();

    double x = xSupplier.get();
    double y = ySupplier.get();
    double theta = SwerveConstants.ROTATION_CONTROLLER.calculate(current.getRadians(), target.getRadians());

    // Apply deadbands
    x = Math.abs(x) > 0.075 ? x : 0.0;
    y = Math.abs(y) > 0.075 ? y : 0.0;
    theta = Math.abs(theta) > 0.075 ? theta : 0.0;

    drive.drive(
      y * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
      x * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
      theta, 
      true, 
      true
    );
  }

  @Override
  public void end(boolean interrupted) {
    //drive.stop();
  }
}
