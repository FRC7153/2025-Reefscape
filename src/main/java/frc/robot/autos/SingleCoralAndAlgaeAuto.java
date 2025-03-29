package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.alignment.AutoLockOnCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.ElevatorState;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.Util;
import frc.robot.util.math.LockOnAlignments;

public class SingleCoralAndAlgaeAuto extends SequentialCommandGroup {
  private final String name;

  /**
   * Drives, places an L4 coral, and picks up an algae.
   * @param drive
   * @param elevator
   * @param manipulator
   * @param led
   * @param pathName Name of initial path to follow from PathPlanner (starting point to reef side).
   * @param reefSide Side of reef to use (index in LockOnAlignments).
   * @param highAlgae Whether to use high algae or low algae intake.
   * @param repeatedly Whether the last command of this auto should be repeatedly (set false to chain)
   */
  public SingleCoralAndAlgaeAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String pathName, int reefSide, boolean highAlgae, boolean repeatedly) {
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
      // Lower elevator to custom state and back away from reef
      new ElevatorToStateCommand(
        elevator, 
        new ElevatorState(
          highAlgae ? ElevatorPositions.ALGAE_HIGH.height() : ElevatorPositions.ALGAE_LOW.height(), 
          ElevatorPositions.STOW.angle()
        )
      ),
      new AutoLockOnCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[reefSide], -1.25, 0.6),
      new WaitCommand(0.1),
      // Go get algae
      new ParallelRaceGroup(
        // Run algae intake
        new AlgaeCommand(elevator, manipulator, led, highAlgae ? ElevatorPositions.ALGAE_HIGH : ElevatorPositions.ALGAE_LOW),
        new SequentialCommandGroup(
          // Get piece
          new AutoLockOnCommand(drive, LockOnAlignments.REEF_CENTER_VECTORS[reefSide], 0.3, 0.55, 1.0, 5.0),
          // Either wait for time, or until the limit switch is pressed
          new WaitCommand(0.9).raceWith(new WaitUntilCommand(elevator::getAlgaeLimitSwitch)),
          // Back up
          new AutoLockOnCommand(drive, LockOnAlignments.REEF_CENTER_VECTORS[reefSide], -1.25, 0.75)
        )
      ),
      // Hold algae
      new PrintCommand("SinglePieceCenterAuto finished!"),
      Util.possiblyRepeatedCommand(new ElevatorToStateCommand(elevator, ElevatorPositions.PROCESSOR), repeatedly)
    );

    // Make sure everything is required
    addRequirements(drive, elevator, manipulator);

    name = String.format(
      "SingleCoralAndAlgaeAuto(%s, %d, %s)", 
      pathName, 
      reefSide,
      highAlgae ? "HIGH" : "LOW"
    );
  }

  /**
   * Drives, places an L4 coral, and picks up an algae.
   * @param drive
   * @param elevator
   * @param manipulator
   * @param led
   * @param pathName Name of initial path to follow from PathPlanner (starting point to reef side).
   * @param reefSide Side of reef to use (index in LockOnAlignments).
   * @param highAlgae Whether to use high algae or low algae intake.
   */
  public SingleCoralAndAlgaeAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator, LED led, String pathName, int reefSide, boolean highAlgae) {
    this(drive, elevator, manipulator, led, pathName, reefSide, highAlgae, true);
  }

  @Override
  public String getName() {
    return name;
  }
}
