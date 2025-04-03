package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class NamedCommandHandler {
  public static void initNamedCommands(Elevator elevator, Manipulator manipulator) {
    // Intake command
    NamedCommands.registerCommand(
      "Intake",
      new SequentialCommandGroup(
        new ElevatorToStateCommand(elevator, ElevatorPositions.INTAKE),
        new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.45), manipulator)
      )
    );

    System.out.println("Initialized PathPlanner's named commands.");
  }
}
