package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.alignment.GoToPointCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.math.AlignmentVector;
import frc.robot.util.math.LockOnAlignments;

/**
 * Various methods to construct multi-coral autos.
 */
public class MultiCoralAutos {
  /**
   * Autonomous that scores 2 L4 Coral on the left side of the field, starting from standard left 
   * position.
   * @param drive
   * @param elevator
   * @param manipulator
   * @return Auto command
   */
  public static Command buildForLeftSide(SwerveDrive drive, Elevator elevator, Manipulator manipulator) {
    return new SequentialCommandGroup(
      // Go to reef, score first coral
      getScoreCoralCommand(drive, elevator, manipulator, "LeftStartToReefJ", LockOnAlignments.REEF_RIGHT_VECTORS[4], true),

      // Get another coral
      getCoralStationCommand(drive, elevator, "ReefJToCoralStation"),

      // Score coral
      getScoreCoralCommand(drive, elevator, manipulator, "CoralStationToReefK", LockOnAlignments.REEF_RIGHT_VECTORS[4], false),

      // End
      new PrintCommand("Left Multi Coral Auto finished"),
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW).repeatedly()
    ).withName("MultiCoralAuto(Left)");
  }

  /**
   * @param drive
   * @param elevator
   * @param manipulator
   * @param goToReefPath
   * @param vector
   * @param resetPosition
   * @return A command group that goes to the reef, lines up, scores, and returns the elevator to STOW.
   */
  private static Command getScoreCoralCommand(SwerveDrive drive, Elevator elevator, Manipulator manipulator, String goToReefPath, AlignmentVector vector, boolean resetPosition) {
    return new SequentialCommandGroup(
      // Drive to reef position
      SwervePaths.getFollowPathCommand(drive, goToReefPath, resetPosition),
      // Elevator up
      new InstantCommand(() -> manipulator.setManipulatorVelocity(-0.8), manipulator),
      new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true, false),
      new WaitCommand(0.7),
      // Go to reef
      new GoToPointCommand(drive, vector, 0.55, 0.8, 3.5),
      // Drop coral
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.35), manipulator),
      new WaitCommand(0.55),
      new InstantCommand(() -> manipulator.setManipulatorVelocity(0.0), manipulator),
      // Lower elevator
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW),
      new WaitCommand(0.25)
    );
  }

  /**
   * @param drive
   * @param elevator
   * @param goToCoralStationPath This path should trigger the Named Command "Intake".
   * @return A command group that goes to the coral station and intakes, then STOWS the elevator.
   */
  private static Command getCoralStationCommand(SwerveDrive drive, Elevator elevator, String goToCoralStationPath) {
    return new SequentialCommandGroup(
      SwervePaths.getFollowPathCommand(drive, goToCoralStationPath), // will raise intake and run manipulator
      new WaitCommand(0.75),
      // Lower intake, return to reef
      new ElevatorToStateCommand(elevator, ElevatorPositions.STOW)
    );
  }
}
