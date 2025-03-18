package frc.robot.autos.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class StallSubsystemsCommand extends Command {
  /**
   * @param elevator
   * @param manipulator
   * @return A Command to schedule a StallSubsystemCommand and NOT wait for it to finish.
   */
  public static Command scheduleInstantly(Elevator elevator, Manipulator manipulator) {
    return new InstantCommand((new StallSubsystemsCommand(elevator, manipulator))::schedule);
  }
  
  /**
   * Holds these subsystems as required so their default commands do not run until autonomous ends.
   * @param elevator
   * @param manipulator
   */
  public StallSubsystemsCommand(Elevator elevator, Manipulator manipulator) {
    addRequirements(elevator, manipulator);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public boolean isFinished() {
    return !DriverStation.isAutonomousEnabled();
  }
}
