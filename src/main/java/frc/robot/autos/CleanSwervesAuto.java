package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CleanSwervesAuto extends Command {
  private final SwerveDrive swerve;

  public CleanSwervesAuto(SwerveDrive swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    swerve.runAllSwervesDutyCycle(0.9);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
