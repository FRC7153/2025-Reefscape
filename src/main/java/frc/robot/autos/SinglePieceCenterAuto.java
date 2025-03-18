package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.alignment.AutoLockOnCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.math.LockOnAlignments;

public class SinglePieceCenterAuto extends SequentialCommandGroup {
  public SinglePieceCenterAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator) {
    super(
      // Drive near reef H position
      SwervePaths.getFollowPathCommand(drive, "CenterStartToReefH"),
      // Put elevator up
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-1.0), manipulator),
      new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true, false),
      new WaitCommand(4),
      // Lock onto reef
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[3], 1.25, 0.5, 2.5),
      // Drop coral
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.1), manipulator),
      new WaitCommand(0.75),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Lower elevator and back away from reef
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[3], -1.25, -1.0, 0.75),
      // TODO more...
      new PrintCommand("SinglePieceCenterAuto finished!")
    );

    // Make sure everything is required
    addRequirements(drive, elevator, manipulator);
  }
}
