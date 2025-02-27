package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator.ElevatorState;

public class AlgaeCommand extends SequentialCommandGroup {
  public AlgaeCommand(Elevator elevator, Manipulator manipulator, ElevatorState position) {
    super(
      new ElevatorToStateCommand(elevator, position),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.3), manipulator),
      new WaitUntilCommand(manipulator::getAlgaeLimitSwitch),
      new ElevatorToStateCommand(elevator, new ElevatorState(position.height(), position.angle() + 0.1)),
      new WaitCommand(1.0),
      new ElevatorToStateCommand(elevator, new ElevatorState(0.1, position.angle() + 0.1)).repeatedly()
    );
    // elevator
  }
}
