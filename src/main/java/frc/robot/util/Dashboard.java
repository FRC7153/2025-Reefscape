package frc.robot.util;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DashboardConstants;

/**
 * Handles some auxiliary Elastic utilities, and a match timer that counts down during real matches
 * and counts up during practice.
 */
public class Dashboard {
  private final DoublePublisher matchTimePub = 
    NetworkTableInstance.getDefault().getDoubleTopic("Elastic/matchTime").publish();
  private final Timer matchTimer = new Timer();

  public Dashboard() {
    // Host Elastic configuration file
    WebServer.start(DashboardConstants.ELASTIC_SERVER_PORT, Filesystem.getDeployDirectory().toPath().resolve("Elastic").toString());

    // Stop the timer (the robot is disabled by default)
    matchTimer.stop();
    matchTimer.reset();
  }

  /**
   * Stops the WebServer hosting Elastic's configuration file if the FMS is attached.
   */
  public void stopWebServerIfFMS() {
    if (DriverStation.isFMSAttached()) {
      System.out.println("Stopped Elastic's WebServer because FMS is attached");
      WebServer.stop(DashboardConstants.ELASTIC_SERVER_PORT);
    }
  }

  /**
   * @return A command to restart the timer. Run when the robot is enabled.
   */
  public Command getRestartTimerCommand() {
    return new InstantCommand(matchTimer::restart).ignoringDisable(true);
  }

  /**
   * @return A command to stop the timer. Run when the robot is disabled.
   */
  public Command getStopTimerCommand() {
    return new InstantCommand(matchTimer::stop).ignoringDisable(true);
  }
  
  public void update() {
    double mt = Timer.getMatchTime();

    if (mt == -1 && !DriverStation.isFMSAttached()) {
      // No FMS match time, count up instead of down
      matchTimePub.set(matchTimer.get());
    } else {
      // Use real match time
      matchTimePub.set(mt);
    }    
  }
}
