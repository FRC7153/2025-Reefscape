package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class DeployClimberCommand extends SequentialCommandGroup {
  /**
   * Deploys the climber outward.
   * @param climber
   */
  public DeployClimberCommand(Climber climber) {
    super(
      new InstantCommand(() -> climber.runClimberPivot(0.75), climber),
      new WaitUntilCommand(() -> climber.getPivotPosition() >= 47), // 47 rotations to out
      new InstantCommand(climber::stopClimber, climber),
      new InstantCommand(climber::setClimberHasDeployedFlag)
    );
  }
}
