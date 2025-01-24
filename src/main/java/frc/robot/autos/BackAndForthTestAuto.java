package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Simple auto that drives forward and backward at a constant rate to test drive base gains.
 */
public class BackAndForthTestAuto extends RepeatCommand {
  public BackAndForthTestAuto(SwerveDrive drive) {
    super(
      new SequentialCommandGroup(
        new InstantCommand(() -> drive.drive(0.0, 2.5, 0.0, true, false)),
        new WaitCommand(1.5),
        new InstantCommand(() -> drive.drive(0.0, -2.5, 0.0, true, false)),
        new WaitCommand(1.5)
      )
    );

    addRequirements(drive);
  }

  @Override
  public String getName() {
    return "BackAndForthAuto";
  }
}
