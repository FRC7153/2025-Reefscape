package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ForTimeCommand extends ParallelRaceGroup {
  private final String name;

  public ForTimeCommand(Command command, double time) {
    super(
      command,
      new WaitCommand(time)
    );

    this.name = command.getName();
  }

  @Override
  public String getName() {
    return name;
  }
}
