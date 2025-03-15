package frc.robot.commands.led;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;

public class SetLEDEnabledCommand extends InstantCommand {
    /**
     * Disables/enabled the LEDs.
     * @param led
     * @param enabled
     */
    public SetLEDEnabledCommand(LED led, boolean enabled) {
        super(() -> led.enableLED(enabled));
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }

    @Override
    public String getName() {
        return "LEDEnabledCommand";
    }
}
