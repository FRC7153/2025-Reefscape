package frc.robot.util;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Alerts that persist for the duration of the robot program once they are triggered.
 */
public class StickyAlerts {
    private static int count = 0;

    /**
     * Creates a sticky ('persistent') Alert. These will not clear until the robot program
     * restarts.
     * @param message
     * @param type
     */
    public static void create(String message) {
        count++;

        @SuppressWarnings("resource")
        Alert alert = new Alert("Sticky", String.format("(%d) %s", count, message), AlertType.kError);
        alert.set(true);
    }

    /** This is a utility class, do not allow instantiation */
    private StickyAlerts() {}
}
