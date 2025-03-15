package frc.robot.commands.led;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class SetLEDColorCommand extends InstantCommand {
    private final String name;

    /**
     * Sets the color to the value from the color supplier once.
     * @param led
     * @param colorSupplier
     */
    public SetLEDColorCommand(LED led, Supplier<Double> colorSupplier) {
        super(() -> led.setColor(colorSupplier.get()), led);

        name = "SetLEDColorCommand(dynamic)";
    }

    /**
     * Sets the color to the value.
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
