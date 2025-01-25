package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Simple auto that drives forward at a constant rate to test drive base gains.
 */
public class SimpleDriveTestAuto extends Command {
  private final SwerveDrive drive;

  public SimpleDriveTestAuto(SwerveDrive drive) {
    this.drive = drive;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    drive.drive(0.0, 2.5, 0.0, true, false);
  }

  @Override
  public void execute() {
    drive.drive(0.0, 2.5, 0.0, true, false);
  }

  @Override
  public void end(boolean interrupted) {
    System.out.printf("SimpleDriveAuto finished (interrupted: %b)", interrupted);
    drive.stop();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public String getName() {
    return "SimpleDriveAuto";
  }
}
