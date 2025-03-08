package frc.robot.commands.sysid;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Climber;

public class ClimberSysIdCommand extends SequentialCommandGroup{
    public ClimberSysIdCommand(Climber climber){
        super(
            new PrintCommand("Climber Q+"),
            new ParallelRaceGroup(
                climber.getClimberPivotRoutine().quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward),
                new WaitUntilCommand(() -> climber.getPivotPosition() >= 0.0)//TODO
            ), 
            new PrintCommand("Climber Q-"),
            new ParallelRaceGroup(
                climber.getClimberPivotRoutine().quasistatic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse),
                new WaitUntilCommand(() -> climber.getPivotPosition() <= 0.0)// TODO
            ),
            new PrintCommand("Climber D+"),
            new ParallelRaceGroup(
                climber.getClimberPivotRoutine().dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kForward),
                new WaitUntilCommand(() -> climber.getPivotPosition() >= 0.0)// TODO
            ),
            new PrintCommand("Climber D-"),
            new ParallelRaceGroup(
                climber.getClimberPivotRoutine().dynamic(edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction.kReverse),
                new WaitUntilCommand(() -> climber.getPivotPosition() <= 0.0)// TODO
            ),
            new PrintCommand("Climber SysId Done"),
            new InstantCommand(() -> climber.stopClimber())
        );
        addRequirements(climber);
    }
}
