package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.subsystems.Elevator;

public class StowCommand extends SequentialCommandGroup {
  /**
   * "Stows" the elevator at the bottom of its range of motion.
   * @param elevator
   */
  public StowCommand(Elevator elevator) {
    super(
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          // Once the elevator reaches the bottom, we can just disable it to not draw power:
          new WaitUntilCommand(() -> elevator.getElevatorHeight() <= 0.15),
          new InstantCommand(elevator::stopElevator)
        ),
        new SequentialCommandGroup(
          // Once the manipulator reaches the top, we can just disable it to not draw power:
          new WaitUntilCommand(() -> elevator.getManipulatorAngle() >= 5.0),
          new InstantCommand(elevator::stopManipulatorPivot)
        )
      )
    );

    addRequirements(elevator);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    // Cancel self when an incoming command needs the elevator.
    return InterruptionBehavior.kCancelSelf;
  }
}
