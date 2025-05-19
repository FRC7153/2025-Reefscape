package frc.robot.util.vision;

import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.dashboard.HardwareFaultTracker;
import libs.LimelightHelpers;

/**
 * @see https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public class Limelight {
  public static enum Version {
    LIMELIGHT_2PLUS(false, true),
    LIMELIGHT_3G(false, false),
    LIMELIGHT_4(true, false);

    /** Whether this version of limelight has an integrated IMU */
    public final boolean integratedIMU;
    /** Whether this limelight has fans. If not, it may overheat if not throttled */
    public final boolean hasFans;

    private Version(boolean integratedIMU, boolean hasFans) {
      this.integratedIMU = integratedIMU;
      this.hasFans = hasFans;
    }
  }

  protected final String cameraName;
  protected final Version version;

  // Network tables
  private final DoubleArraySubscriber statsSub;
  private final DoubleSubscriber heartbeatSub;
  private final DoublePublisher pipelinePub;
  private final Alert notConnectedAlert;

  // Stats
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatTS = 0.0;
  private boolean alive = false;

  // Logging
  private final DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog;
  private final BooleanLogEntry isAliveLog;

  /**
   * @param name Host Camera ID
   */
  public Limelight(String name, Version version) {
    cameraName = name;
    this.version = version;

    notConnectedAlert = new Alert(String.format("Limelight %s is not alive", name), AlertType.kWarning);

    // Get NT topics
    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);
    
    statsSub = cameraTable.getDoubleArrayTopic("hw").subscribe(new double[4]);
    heartbeatSub = cameraTable.getDoubleTopic("hb").subscribe(-1.0);

    pipelinePub = cameraTable.getDoubleTopic("pipeline").publish();

    // Init logging
    String logName = String.format("Limelight/%s/", name);

    fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
    cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpu_temp", "f");
    ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram", "%");
    tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");
    isAliveLog = new BooleanLogEntry(DataLogManager.getLog(), logName + "is_alive");
  }

  /**
   * Sets the pipeline index.
   * @param pipeline The pipeline index.
   */
  public void setPipeline(int pipeline) {
    pipelinePub.set(pipeline);
  }

  /**
   * Logs diagnostics data
   */
  public void log() {
    // Get stats
    double[] stats = statsSub.get();

    if (alive && stats.length == 4) {
      fpsLog.append(stats[0]);
      cpuTempLog.append(stats[1]);
      ramLog.append(stats[2]);
      tempLog.append(stats[3]);
    }
  }

  /**
   * Checks if this limelight is alive
   */
  public void checkHardware() {
    // Check heartbeat
    double newHeartbeat = heartbeatSub.get();

    if (newHeartbeat == -1.0) {
      // No heartbeat
      alive = false;
    } else if (newHeartbeat != lastHeartbeat) {
      // New heartbeat
      lastHeartbeat = newHeartbeat;
      lastHeartbeatTS = Timer.getFPGATimestamp();
      alive = true;
    } else if (Timer.getFPGATimestamp() - lastHeartbeatTS > 1.5) {
      // No recent or new heartbeats
      alive = false;
    }

    //notConnectedAlert.set(!alive);
    HardwareFaultTracker.checkFault(notConnectedAlert, !alive);
    isAliveLog.append(alive);
  }

  /**
   * Takes a photo with the limelight
   * @param snapshotName Name to save the photo.
   */
  public void takeSnapshot(String snapshotName) {
    LimelightHelpers.takeSnapshot(cameraName, snapshotName).thenAccept((Boolean success) -> {
      if (success) {
        // Photo was successful
        System.out.printf("Successfully snapped '%s' with limelight '%s'\n", snapshotName, cameraName);
      } else {
        // Photo failed
        System.out.printf("Failed to snap '%s' with limelight '%s'\n", snapshotName, cameraName);
      }
    });
  }

  /**
   * @return If the limelight is returning a heartbeat.
   */
  public boolean isAlive() {
    return alive;
  }
  
  /**
   * @return The version of this limelight.
   */
  public Version getVersion() {
    return version;
  }
}
