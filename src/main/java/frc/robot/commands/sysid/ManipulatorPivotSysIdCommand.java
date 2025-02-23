package frc.robot.commands.sysid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;

public class ManipulatorPivotSysIdCommand extends SequentialCommandGroup{
  public ManipulatorPivotSysIdCommand(Elevator elevator){
    super(
      new PrintCommand("Manipulator Pivot Q+"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine().quasistatic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() >= 0.36)
      ), 
      new PrintCommand("Manipulator Pivot Q-"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine().quasistatic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() <= -0.24)
      ),
      new PrintCommand("Manipulator Pivot D+"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine().dynamic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() >= 0.36)
      ),
      new PrintCommand("Manipulator Pivot D-"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine().dynamic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() <= -0.24)
      ),
      new PrintCommand("Manipulator Pivot Done"),
      new InstantCommand(() -> elevator.stopManipulatorPivot())
      );

        addRequirements();
    }
}
