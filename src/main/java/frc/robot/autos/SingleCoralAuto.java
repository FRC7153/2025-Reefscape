package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.alignment.AutoLockOnCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.math.LockOnAlignments;

public class SingleCoralAuto extends SequentialCommandGroup {
  private final String name;

  /**
   * Drives and places an L4 coral.
   * @param drive
   * @param elevator
   * @param manipulator
   * @param led
   * @param pathName Name of initial path to follow from PathPlanner (starting point to reef side).
   * @param reefSide Side of reef to use (index in LockOnAlignments).
   * @param highAlgae Whether to use high algae or low algae intake.
   * @param repeatedly Whether the last command of this auto should be repeatedly (set false to chain)
   */
  public SingleCoralAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String pathName, int reefSide) {
    super(
      // Drive near reef position
      SwervePaths.getFollowPathCommand(drive, pathName, true),
      // Put elevator up
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.8), manipulator),
      new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true, false),
      new WaitCommand(0.85),
      // Lock onto reef
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[reefSide], 0.225, 0.55, 0.8, 3.5),
      // Drop coral
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.1), manipulator),
      new WaitCommand(0.75),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Lower elevator and back away from reef
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[reefSide], -1.25, 0.6),
      new PrintCommand("SingleCoralAuto finished"),
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW).repeatedly()
    );

    // Make sure everything is required
    addRequirements(drive, elevator, manipulator);

    name = String.format(
      "SingleCoralAuto(%s, %d)", 
      pathName, 
      reefSide
    );
  }

  @Override
  public String getName() {
    return name;
  }
}
