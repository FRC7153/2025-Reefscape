package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;

public class SinglePieceAuto extends SequentialCommandGroup {
  public SinglePieceAuto(SwerveDrive drive, Elevator elevator) {
    super(
      SwervePaths.getFollowPathCommand(drive, "Test Auto 1", true),
      new ParallelCommandGroup(
        new ElevatorToStateCommand(elevator, ElevatorPositions.L2).repeatedly(),
        new WaitCommand(3.0)
      )
    );
  }
}
