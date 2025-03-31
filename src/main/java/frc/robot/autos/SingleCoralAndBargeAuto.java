package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;

public class SingleCoralAndBargeAuto extends SequentialCommandGroup {
  private final String name;

  public SingleCoralAndBargeAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String startToReefPath, int reefSide, boolean highAlgae, String reefToBargePath) {
    super(
      // Place L4 and grab algae
      new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, startToReefPath, reefSide, highAlgae, false),
      new WaitCommand(0.35),
      // Go to barge
      SwervePaths.getFollowPathCommand(drive, reefToBargePath),
      // Raise elevator
      new ElevatorToStateCommand(elevator, ElevatorPositions.BARGE),
      new WaitUntilCommand(() -> elevator.getElevatorHeight() >= 4.375).raceWith(new WaitCommand(3.0)),
      new WaitCommand(0.35),
      // Shoot algae
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.55), manipulator),
      new WaitCommand(0.6),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Bring elevator down
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW).repeatedly()
    );

    addRequirements(drive, elevator, manipulator);

    name = String.format(
      "SingleCoralAndBargeAuto(%d, %s)", 
      reefSide,
      highAlgae ? "HIGH" : "LOW"
    );
  }

  @Override
  public String getName() {
    return name;
  }
}
