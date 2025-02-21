package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorToStateCommand extends InstantCommand {
  /**
   * InstantCommand to set the state of the elevator.
   * @param elevator
   * @param state
   */
  public ElevatorToStateCommand(Elevator elevator, ElevatorState state) {
    super(() -> elevator.setState(state), elevator);
  }
}
