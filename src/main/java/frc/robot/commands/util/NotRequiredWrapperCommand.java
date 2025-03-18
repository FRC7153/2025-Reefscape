package frc.robot.commands.util;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;

public class NotRequiredWrapperCommand extends Command {
  // List of all currently running NotRequiredWrapperCommands
  private static final ArrayList<NotRequiredWrapperCommand> running = new ArrayList<>();

  /**
   * Stops all currently scheduled NotRequiredWrapperCommands.
   */
  public static void stopAllWrapperCommands() {
    int count = running.size();

    for (NotRequiredWrapperCommand c : running) {
      c.cancel();
    }

    System.out.printf("Canceled %d NotRequiredWrapperCommands\n", count);
  }

  private final Command command;

  /**
   * Wrapper command that does not require any of the original command's requirements (use this in
   * auto)
   * @param command
   */
  public NotRequiredWrapperCommand(Command command) {
    this.command = command;
  }

  @Override
  public void initialize() {
    running.add(this);
    command.initialize();
  }

  @Override
  public void execute() {
    command.execute();
  }

  @Override
  public void end(boolean interrupted) {
    command.end(interrupted);
    running.remove(this);
  }

  @Override
  public boolean isFinished() {
    return command.isFinished();
  }

  @Override
  public String getName() {
    return String.format("NotRequiredWrapper(%s)", command.getName());
  }
}
