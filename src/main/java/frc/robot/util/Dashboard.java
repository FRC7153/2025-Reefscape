package frc.robot.util;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.DashboardConstants;

/**
 * Handles some auxiliary Elastic utilities.
 */
public class Dashboard {
  private final DoublePublisher matchTimePub = 
    NetworkTableInstance.getDefault().getDoubleTopic("Elastic/matchTime").publish();

  public Dashboard() {
    // Host Elastic configuration file
    WebServer.start(DashboardConstants.ELASTIC_SERVER_PORT, Filesystem.getDeployDirectory().toPath().resolve("Elastic").toString());
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
  
  public void update() {
    matchTimePub.set(Timer.getMatchTime());
  }
}
