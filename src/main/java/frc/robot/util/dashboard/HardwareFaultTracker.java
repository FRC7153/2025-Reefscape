package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Tracks whether a hardware fault ever occurs
 */
public class HardwareFaultTracker {
  private static final Alert hasFaultOccurredAlert = 
    new Alert("A hardware fault has been reported. Check logs.", AlertType.kInfo);

  // Don't track faults from before the robot program has started booting
  private static boolean hasRobotProgramStarted = false;

  /**
   * Starts tracking faults.
   */
  public static void robotProgramHasStarted() {
    hasRobotProgramStarted = true;
    System.out.println("Hardware fault tracker has started.");
  }

  /**
   * If a fault has ocurred, sets the given alert and a global persistent alert.
   * @param alert The alert specific to the hardware being checked.
   * @param fault Whether a fault has occurred.
   */
  public static void checkFault(Alert alert, boolean fault) {
    alert.set(fault);

    if (fault && hasRobotProgramStarted) {
      hasFaultOccurredAlert.set(true);
    }
  }
}
