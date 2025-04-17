package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;

public class SingleCoralAndBargeAuto extends SequentialCommandGroup {
  public SingleCoralAndBargeAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led) {
    super(
      // Place coral, grab algae
      new SingleCoralAndAlgaeAuto(drive, elevator, manipulator, led, "CenterStartToReefH", 7, 3, false, false),
      // Drive to barge
      SwervePaths.getFollowPathCommand(drive, "ReefHToBarge"),
      // Barge height
      new ElevatorToStateCommand(elevator, ElevatorPositions.BARGE),
      new WaitCommand(1.7),
      // Outtake
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.65), manipulator),
      new WaitCommand(0.75),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Elevator down, back up
      new InstantCommand(() -> drive.drive(0.0, -2.5, 0.0, true, true)),
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new WaitCommand(1.1),
      new InstantCommand(drive::stop),
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW).repeatedly()
    );

    addRequirements(drive, elevator, manipulator);
  }
}
