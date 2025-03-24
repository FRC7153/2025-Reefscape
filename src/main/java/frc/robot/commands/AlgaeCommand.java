package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.util.dashboard.NotificationCommand;
import libs.Elastic.Notification.NotificationLevel;

public class AlgaeCommand extends SequentialCommandGroup {
  public AlgaeCommand(Elevator elevator, Manipulator manipulator, LED led, ElevatorState position) {
    super(
      // Go to Algae intake position, run intake
      new ElevatorToStateCommand(elevator, position),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.3), manipulator),
      // Wait until limit switch pressed
      new WaitUntilCommand(elevator::getAlgaeLimitSwitch),
      // Clamp
      new ElevatorToStateCommand(elevator, new ElevatorState(position.height(), position.angle() + 0.05)),
      new InstantCommand(led.solidYellowTwoSeconds::schedule),
      new NotificationCommand("Algae Intake Success", "Elevator will retract in 1 second", NotificationLevel.INFO),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      new WaitCommand(1.0),
      // Retract elevator, hold here until this command is canceled
      new ElevatorToStateCommand(elevator, new ElevatorState(0.1, position.angle() + 0.05)).repeatedly()
    );
  }
}
