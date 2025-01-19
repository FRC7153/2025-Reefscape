package frc.robot.util;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * Handles some auxiliary Elastic utilities.
 */
public class Dashboard {
  private final DoublePublisher matchTimePub = 
    NetworkTableInstance.getDefault().getDoubleTopic("Elastic/matchTime").publish();

  public Dashboard() {
    // Host Elastic configuration file
    WebServer.start(5800, Filesystem.getDeployDirectory().toPath().resolve("Elastic").toString());
  }
  
  public void update() {
    matchTimePub.set(Timer.getMatchTime());
  }
}
