package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.Constants.LEDColors;
import frc.robot.subsystems.LED;

public class FlashLEDCommand extends WrapperCommand {
  private final String name;

  /**
   * Flashes the LED numFlashes time. Cancels all incoming commands.
   * @param led
   * @param color
   * @param numFlashes
   */
  public FlashLEDCommand(LED led, double color, int numFlashes, double flashTime) {
    super(
      numFlashes > 0 ?
      // Flash, then run again if flashes > 0
      new SequentialCommandGroup(
        new SetLEDColorCommand(led, color),
        new WaitCommand(flashTime),
        new SetLEDColorCommand(led, LEDColors.BLACK),
        new WaitCommand(flashTime),
        new FlashLEDCommand(led, color, numFlashes-1, flashTime)
      ) : 
      // Just do nothing if flashes <= 0
      new PrintCommand("LED finished flashing")
    );

    this.name = String.format("FlashLEDCommand(%.3f, %d)", color, numFlashes);
  }

  public FlashLEDCommand(LED led, double color, int numFlashes) {
    this(led, color, numFlashes, 0.08);
  }

  @Override
  public InterruptionBehavior getInterruptionBehavior() {
    return InterruptionBehavior.kCancelIncoming;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public String getName() {
    return name;
  }
}
