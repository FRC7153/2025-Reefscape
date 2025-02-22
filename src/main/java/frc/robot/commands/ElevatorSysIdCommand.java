package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.Elevator;

public class ElevatorSysIdCommand extends SequentialCommandGroup {
  /**
   * Runs all the elevator SysId routines back-to-back.
   * @param elevator
   */
  public ElevatorSysIdCommand(Elevator elevator) {
    super(
      new PrintCommand("Elevator Q+"),
      new ParallelRaceGroup(
        elevator.getElevatorRoutine(elevator).quasistatic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getElevatorHeight() >= 3.9) 
      ),
      new PrintCommand("Elevator Q-"),
      new ParallelRaceGroup(
        elevator.getElevatorRoutine(elevator).quasistatic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getElevatorHeight() < 0.1) 
      ),
      new PrintCommand("Elevator D+"),
      new ParallelRaceGroup(
        elevator.getElevatorRoutine(elevator).dynamic(Direction.kForward),
        new WaitUntilCommand(() -> elevator.getElevatorHeight() >= 3.9) 
      ),
      new PrintCommand("Elevator D-"),
      new ParallelRaceGroup(
        elevator.getElevatorRoutine(elevator).dynamic(Direction.kReverse),
        new WaitUntilCommand(() -> elevator.getElevatorHeight() <= 0.1) 
      ),
      new PrintCommand("Done"),
      new InstantCommand(() -> elevator.stopElevator(), elevator)
    );

    addRequirements(elevator);
  }
}
