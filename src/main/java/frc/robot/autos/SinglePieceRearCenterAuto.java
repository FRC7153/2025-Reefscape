package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.alignment.AutoLockOnCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.math.LockOnAlignments;

public class SinglePieceRearCenterAuto extends SequentialCommandGroup {
  /**
   * 
   * @param drive
   * @param elevator
   * @param manipulator
   * @param led
   */
  public SinglePieceRearCenterAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String pathName) {
    super(
      // Drive near reef position
      SwervePaths.getFollowPathCommand(drive, pathName),
      // Put elevator up
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.8), manipulator),
      new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true, false),
      new WaitCommand(0.85),
      // Lock onto reef
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[3], 0.225, 0.6, 0.8, 5.0),
      // Drop coral
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.1), manipulator),
      new WaitCommand(0.75),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Lower elevator and back away from reef
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[3], -1.25, 0.6),
      new WaitCommand(0.1),
      // Go get algae
      new ParallelRaceGroup(
        // Run algae intake
        new AlgaeCommand(elevator, manipulator, led, ElevatorPositions.ALGAE_HIGH),
        new SequentialCommandGroup(
          // Get piece
          new AutoLockOnCommand(drive, LockOnAlignments.REEF_CENTER_VECTORS[3], 0.3, 0.6, 1.0, 5.0),
          new WaitCommand(0.1),
          // Back up
          new AutoLockOnCommand(drive, LockOnAlignments.REEF_CENTER_VECTORS[3], -1.25, 0.75)
        )
      ),
      // Hold algae
      new PrintCommand("SinglePieceCenterAuto finished!"),
      new ElevatorToStateCommand(elevator, ElevatorPositions.PROCESSOR).repeatedly()
    );

    // Make sure everything is required
    addRequirements(drive, elevator, manipulator);
  }
}
