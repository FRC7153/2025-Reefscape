package frc.robot.autos;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.subsystems.Elevator;

public class AutoCommandHandler {
  public static void initNamedCommands(Elevator elevator) {
    NamedCommands.registerCommand("PrintTest", new PrintCommand("..."));
    NamedCommands.registerCommand("ElevatorL2", new ElevatorToStateCommand(elevator, ElevatorPositions.L2).repeatedly());
    
    System.out.println("Auto NamedCommands have been initialized");
  }

  /** Utility class, prevent instantiation. */
  private AutoCommandHandler() {}
}
