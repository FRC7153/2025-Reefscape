package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;

public class ManipulatorPivotSysIdCommand extends SequentialCommandGroup{
  private static double topManipulatorSetpoint;//TODO
  private static double bottomManipulatorSetpoint;//TODO
      
  public ManipulatorPivotSysIdCommand(Elevator elevator){
    super(
      new PrintCommand("Manipulator Pivot Q+"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine(elevator).quasistatic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() >= topManipulatorSetpoint)
      ), 
      new PrintCommand("Manipulator Pivot Q-"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine(elevator).quasistatic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() <= bottomManipulatorSetpoint)
      ),
      new PrintCommand("Manipulator Pivot D+"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine(elevator).dynamic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() >= topManipulatorSetpoint)
      ),
      new PrintCommand("Manipulator Pivot D-"),
      new ParallelRaceGroup(
        elevator.getManipulatorPivotRoutine(elevator).dynamic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getManipulatorAngle() <= bottomManipulatorSetpoint)
      ),
      new PrintCommand("Manipulator Pivot Done"),
      new InstantCommand(() -> elevator.stopManipulatorPivot())
      );

        addRequirements(elevator);
    }
}
