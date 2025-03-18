package frc.robot.commands.util;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class ForTimeCommand extends ParallelRaceGroup {
  private final String name;

  /**
   * Runs a command for a certain amount of time.
   * @param command The command to run.
   * @param time The time, in seconds, to run for.
   * @param repeat Whether the command should be repeated or not. If not, and it finishes early, this
   * command will finish.
   */
  public ForTimeCommand(Command command, double time, boolean repeat) {
    super(
      repeat ? command.repeatedly() : command,
      new WaitCommand(time)
    );

    this.name = command.getName();
  }

  /**
   * Runs a command for a certain amount of time repeatedly.
   * @param command The command to run.
   * @param time The time, in seconds, to run for.
   */
  public ForTimeCommand(Command command, double time) {
    this(command, time, true);
  }

  @Override
  public String getName() {
    return name;
  }
}
