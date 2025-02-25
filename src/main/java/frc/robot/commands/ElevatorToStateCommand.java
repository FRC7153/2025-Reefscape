package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class ElevatorToStateCommand extends WrapperCommand {
  /**
   * InstantCommand to set the state of the elevator.
   * @param elevator
   * @param state
   * @param delayArm Whether to wait 0.5 seconds before moving the arm. If true, this runs until
   * canceled.
   */
  public ElevatorToStateCommand(Elevator elevator, ElevatorState state, boolean delayArm) {
    super(
      delayArm ?
      // If delay arm:
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorPosition(state.height()), elevator),
        new WaitCommand(0.5),
        new InstantCommand(() -> elevator.setState(state), elevator).repeatedly()
      ) :
      // Else, run immediately:
      new InstantCommand(() -> elevator.setState(state), elevator)
    );
  }

  /**
   * InstantCommand to set the state of the elevator once.
   * @param elevator
   * @param state
   */
  public ElevatorToStateCommand(Elevator elevator, ElevatorState state) {
    this(elevator, state, false);
  }
}
