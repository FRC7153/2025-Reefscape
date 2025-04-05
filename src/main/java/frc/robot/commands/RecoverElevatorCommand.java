package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.Elevator;

public class RecoverElevatorCommand extends SequentialCommandGroup {
  /**
   * Recovers the elevator by slowly lowering it, regardless of height.
   * @param elevator
   */
  public RecoverElevatorCommand(Elevator elevator) {
    super(
      new PrintCommand("Beginning elevator recovery"),
      new InstantCommand(() -> elevator.setManipulatorPivotPosition(ElevatorPositions.STOW.angle()), elevator),
      new InstantCommand(() -> elevator.setHeightDutyCycle(-0.075), elevator).repeatedly()
    );

    addRequirements(elevator);
  }
}
