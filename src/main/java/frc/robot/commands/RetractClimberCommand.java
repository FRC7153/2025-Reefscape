package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class RetractClimberCommand extends SequentialCommandGroup {
  /**
   * Retracts the climber, pulling the robot upwards.
   * @param climber
   */
  public RetractClimberCommand(Climber climber) {
    super(
      new InstantCommand(() -> climber.runClimberPivot(-0.05), climber),
      new InstantCommand(() -> climber.runClimberWinch(-0.85), climber),
      new InstantCommand(() -> climber.setBrakeMode(false, true), climber),
      new WaitUntilCommand(() -> climber.getPivotPosition() <= 11), // 11 rots to perp with ground
      new InstantCommand(climber::stopClimber, climber)
    );
  }
}
