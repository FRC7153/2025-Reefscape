package frc.robot.autos.commands;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.util.NotRequiredWrapperCommand;
import frc.robot.subsystems.Elevator;

public class AutoCommandHandler {
  /**
   * Registers all auto named commands.
   * @param elevator
   */
  public static void initNamedCommands(Elevator elevator) {
    NamedCommands.registerCommand("PrintTest", new PrintCommand("Auto PrintTest worked."));

    NamedCommands.registerCommand(
      "ElevatorL4", 
      new NotRequiredWrapperCommand(new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true, false)));

    NamedCommands.registerCommand(
      "StowElevator", 
      new NotRequiredWrapperCommand(new ElevatorToStateCommand(elevator, ElevatorPositions.STOW)));
    
    System.out.println("Auto NamedCommands have been initialized");
  }

  /** Utility class, prevent instantiation. */
  private AutoCommandHandler() {}
}
