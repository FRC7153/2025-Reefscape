package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.autos.commands.StallSubsystemsCommand;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.util.ForTimeCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;

public class SinglePieceCenterAuto extends SequentialCommandGroup {
  public SinglePieceCenterAuto(SwerveDrive drive, Elevator elevator, Manipulator manipulator) {
    super(
      // Begin stalling subsystems
      StallSubsystemsCommand.scheduleInstantly(elevator, manipulator),
      // Drive near reef H position
      SwervePaths.getFollowPathCommand(drive, "CenterStartToReefH"),
      new ParallelCommandGroup(
        new ProxyCommand(new ForTimeCommand(new ElevatorToStateCommand(elevator, ElevatorPositions.L4).repeatedly(), 4.0)),
        new SequentialCommandGroup(
          new WaitCommand(1.75),
          new ProxyCommand(new ForTimeCommand(new ManipulatorCommand(manipulator, 0.1), 0.5))
        )
      )
      
      /*,
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new ElevatorToStateCommand(elevator, ElevatorPositions.L4).forSeconds(1.25)
        ),
        new ParallelRaceGroup(
          new SequentialCommandGroup(
            new WaitCommand(0.5),
            new GoToPointCommand(drive, LockOnAlignments.REEF_RIGHT_VECTORS[3].getTargetAsPose2d())
          ),
          new WaitCommand(1.0)
        )
      ),
      SwervePaths.getFollowPathCommand(drive, "Reef4ToBack", false)*/
    );
  }
}
