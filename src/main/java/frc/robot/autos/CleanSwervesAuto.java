package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CleanSwervesAuto extends Command {
  private final SwerveDrive swerve;

  /**
   * Auto that spins all swerves at a certain duty cycle to dislodge anything stuck in the gears
   * or belts.
   * @param swerve
   */
  public CleanSwervesAuto(SwerveDrive swerve) {
    this.swerve = swerve;
  }

  @Override
  public void initialize() {
    System.out.println("Starting CleanSwervesAuto!");
    swerve.runAllSwervesDutyCycle(0.9);
  }

  @Override
  public void end(boolean interrupted) {
    swerve.stop();
  }
}
