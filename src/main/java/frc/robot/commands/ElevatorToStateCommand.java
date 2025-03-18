package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.util.Util;

public class ElevatorToStateCommand extends WrapperCommand {
  /**
   * InstantCommand to set the state of the elevator.
   * @param elevator
   * @param state
   * @param delayArm Whether to wait 0.5 seconds before moving the arm. If true, this runs until
   * canceled.
   */
  public ElevatorToStateCommand(Elevator elevator, ElevatorState state, boolean delayArm, boolean repeat) {
    super(
      delayArm ?
      // If delay arm:
      new SequentialCommandGroup(
        new InstantCommand(() -> elevator.setElevatorPosition(state.height()), elevator),
        new WaitCommand(0.5),
        Util.possiblyRepeatedCommand(new InstantCommand(() -> elevator.setState(state), elevator), repeat)
      ) :
      // Else, run both:
      Util.possiblyRepeatedCommand(new InstantCommand(() -> elevator.setState(state), elevator), repeat)
    );
  }

  /**
   * Command to set the state of the elevator
   * @param elevator
   * @param state
   * @param delayArm Whether to wait 0.5 seconds before moving the arm. If true, this runs until
   * canceled. Otherwise, it's an instant command.
   */
  public ElevatorToStateCommand(Elevator elevator, ElevatorState state, boolean delayArm) {
    this(elevator, state, delayArm, delayArm);
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
