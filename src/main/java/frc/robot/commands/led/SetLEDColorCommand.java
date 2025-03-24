package frc.robot.commands.led;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.LED;

public class SetLEDColorCommand extends InstantCommand {
  /**
   * Sets the LED to a certain color for a certain amount of time.
   * @param led
   * @param color
   * @param time Time to set for, in seconds
   * @return The command to run
   */
    public static Command forTime(LED led, double color, double time) {
      return new ParallelRaceGroup(
        new SetLEDColorCommand(led, color).repeatedly(),
        new WaitCommand(time)
      ).withName(String.format("SetLEDColorCommand(%.3f, timed %.3f)", color, time));
    }

    private final String name;

    /**
     * Sets the color to the value from the color supplier until canceled.
     * @param led
     * @param colorSupplier
     */
    public SetLEDColorCommand(LED led, Supplier<Double> colorSupplier) {
        super(() -> led.setColor(colorSupplier.get()), led);

        name = "SetLEDColorCommand(dynamic)";
    }

    /**
     * Sets the color to the value until canceled.
     * @param led
     * @param color
     */
    public SetLEDColorCommand(LED led, double color) {
        super(() -> led.setColor(color), led);

        name = String.format("SetLEDColorCommand(%.3f)", color);
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
