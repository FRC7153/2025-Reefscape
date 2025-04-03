package frc.robot.util.dashboard;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Timer;

/**
 * Tracks whether a hardware fault ever occurs
 */
public class HardwareFaultTracker {
  private static final Alert hasFaultOccurredAlert = 
    new Alert("A hardware fault has been reported. Check logs.", AlertType.kInfo);

  // Don't track faults from before the robot program has started booting
  private static double robotProgramStartTime = -1.0;

  /**
   * Starts tracking faults.
   */
  public static void robotProgramHasStarted() {
    robotProgramStartTime = Timer.getFPGATimestamp();
    System.out.printf("Hardware fault tracker has started at %f.\n", robotProgramStartTime);
  }

  /**
   * If a fault has ocurred, sets the given alert and a global persistent alert.
   * @param alert The alert specific to the hardware being checked.
   * @param fault Whether a fault has occurred.
   */
  public static void checkFault(Alert alert, boolean fault) {
    alert.set(fault);

    // Only log if robot program has been running for at least 10 seconds
    if (fault && robotProgramStartTime != -1.0 && Timer.getFPGATimestamp() - robotProgramStartTime >= 10.0) {
      hasFaultOccurredAlert.set(true);
    }
  }
}
