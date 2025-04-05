package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.LED;

public class RetractClimberCommand extends ConditionalCommand {
  /**
   * Retracts the climber, pulling the robot upwards.
   * @param climber
   */
  public RetractClimberCommand(Climber climber, LED led) {
    super(
      // If has deployed flag is true, climb:
      new SequentialCommandGroup(
        new InstantCommand(() -> climber.runClimberPivot(-0.05), climber),
        new InstantCommand(() -> climber.runClimberWinch(-0.85), climber),
        new InstantCommand(() -> climber.setBrakeMode(false, true), climber),
        new WaitUntilCommand(() -> climber.getPivotPosition() <= 0.19), // rots to perp with ground
        new InstantCommand(climber::stopClimber, climber),
        new InstantCommand(led.flashGreenThreeTimes::schedule)
      ),
      // If has deployed flag is false, do nothing
      new PrintCommand("Will not climb because climber deploy flag has not been set to true yet."),
      // Check if flag is true
      climber::getClimberHasDeployedFlag
    );
  }
}
