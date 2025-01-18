package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class GoToPointCommand extends Command {
  private final PIDController pidXController = new PIDController(0.1, 0, 0);
  private final PIDController pidYController = new PIDController(0.1, 0, 0);
  private final PIDController pidThetaController = new PIDController(0.1, 0, 0);

  private final SwerveDrive drive;
  private final Pose2d target;

  /**
   * Command that goes to a certain point.
   * @param drive
   * @param target Alliance relative target position
   */
  public GoToPointCommand(SwerveDrive drive, Pose2d target) {
    this.drive = drive;
    this.target = target;

    pidXController.setSetpoint(target.getX());
    pidYController.setSetpoint(target.getY());
    pidThetaController.setSetpoint(target.getRotation().getDegrees());

    pidThetaController.enableContinuousInput(0, 360);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    pidXController.reset();
    pidYController.reset();
    pidThetaController.reset();
  }

  @Override
  public void execute() {
    Pose2d pose = drive.getAllianceRelativePose();
    drive.drive(
        pidYController.calculate(pose.getY()),
        pidXController.calculate(pose.getX()),
        pidThetaController.calculate(pose.getRotation().getDegrees()),
        true,
        false);
  }

  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      drive.drive(0, 0, 0, false, false);
    }
  }

  @Override
  public String getName() {
    return "goToPoint " + target.toString();
  }

}
