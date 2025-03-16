package frc.robot.commands.led;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.LED;

public class SetLEDColorCommand extends RepeatCommand {
    private final String name;

    /**
     * Sets the color to the value from the color supplier until canceled.
     * @param led
     * @param colorSupplier
     */
    public SetLEDColorCommand(LED led, Supplier<Double> colorSupplier) {
        super(new InstantCommand(() -> led.setColor(colorSupplier.get()), led));

        name = "SetLEDColorCommand(dynamic)";
    }

    /**
     * Sets the color to the value until canceled.
     * @param led
     * @param color
     */
    public SetLEDColorCommand(LED led, double color) {
        super(new InstantCommand(() -> led.setColor(color), led));

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
