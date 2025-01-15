package frc.robot.util;

import java.util.EnumSet;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveOdometry;

public class Limelight {
  private String cameraName;
  private SwerveOdometry odometry;

  // Network tables
  private DoubleArraySubscriber poseSub, statsSub, stdDevSub;
  private DoubleSubscriber heartbeatSub;

  // Heartbeat Cache
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatTS = 0.0;
  private boolean alive = false;

  // Logging
  private DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog;

  /**
   * @param name Host Camera ID
   */
  public Limelight(String name, SwerveOdometry odometry) {
    name = cameraName;
    this.odometry = odometry;

    // Network Table
    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);

    poseSub = cameraTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[11]);
    stdDevSub = cameraTable.getDoubleArrayTopic("stdDevs").subscribe(new double[12]);
    
    statsSub = cameraTable.getDoubleArrayTopic("hw").subscribe(new double[4]);
    heartbeatSub = cameraTable.getDoubleTopic("hb").subscribe(-1.0);

    // Enforce Pipeline
    cameraTable.getIntegerTopic("pipeline").publish().set(1);

    // Init logging
    String logName = String.format("Limelight/%s/", name);

    fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
    cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpu_temp", "f");
    ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram", "%");
    tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");

    // Limelight Listener
    NetworkTableInstance.getDefault().addListener(
      poseSub, 
      EnumSet.of(NetworkTableEvent.Kind.kValueAll),
      this::processData
    ); 
  }

  private void processData(NetworkTableEvent event) {
    if (!event.valueData.value.isDoubleArray()) {
      DriverStation.reportError("Received data that wasn't double[]", false);
      return;
    }

    double[] data = event.valueData.value.getDoubleArray();
    double[] stdDevs = stdDevSub.get();

    if (data.length != 11 || stdDevs.length != 12) {
      DriverStation.reportError("Invalid length of limelight data!", false);
      return;
    }

    Pose2d receivedPose = new Pose2d(
      new Translation2d(data[0], data[1]),
      Rotation2d.fromDegrees(data[5])
    );

    double timestamp = event.valueData.value.getTime() - data[6];

    odometry.addVisionMeasurement(
      receivedPose, 
      timestamp,
      VecBuilder.fill(stdDevs[6], stdDevs[7], stdDevs[12])
    );

    var x = VecBuilder.fill(0, 0, 0);
  }

  /**
   * Caches the latest results, checks the heartbeat, and logs stats.
   * Called periodically
   */
  public void refresh() {
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
    } else if (Timer.getFPGATimestamp() - lastHeartbeatTS > 0.75) {
      // No recent or new heartbeats
      alive = false;
    }

    // Get stats
    double[] stats = statsSub.get();

    if (stats.length != 4) {
      DriverStation.reportWarning("Invalid limelight stats length", false);
    } else {
      fpsLog.append(stats[0]);
      cpuTempLog.append(stats[1]);
      ramLog.append(stats[2]);
      tempLog.append(stats[3]);
    }

    LimelightHelpers.SetRobotOrientation(cameraName,
        odometry.getFieldRelativePosition().getRotation().getDegrees(),
        odometry.getYawRate(), 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * @return If the limelight is returning a heartbeat.
   */
  public boolean isAlive() {
    return alive;
  }
}
