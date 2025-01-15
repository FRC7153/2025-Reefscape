package frc.robot.util;

import java.util.EnumSet;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.swerve.SwerveOdometry;

/**
 * @see https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public class Limelight {
  private final String cameraName;
  private final SwerveOdometry odometry;

  // Network tables
  private final DoubleArraySubscriber poseSub, statsSub, stdDevSub;
  private final DoubleSubscriber heartbeatSub;
  private final DoubleArrayPublisher orientationPub;
  private final Alert notConnectedAlert;

  // Stats
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatTS = 0.0;
  private boolean alive = false;
  private int frameCount = 0;

  // Logging
  private final DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog;
  private final IntegerLogEntry frameCountLog;

  // Cached values
  private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 0);
  private final double[] orientation = new double[6];

  /**
   * @param name Host Camera ID
   */
  public Limelight(String name, SwerveOdometry odometry) {
    cameraName = name;
    this.odometry = odometry;
    notConnectedAlert = new Alert(String.format("Limelight %s is not alive", name), AlertType.kError);

    // Get NT topics
    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);

    poseSub = cameraTable.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[0]);
    stdDevSub = cameraTable.getDoubleArrayTopic("stdDevs").subscribe(new double[0]);
    orientationPub = cameraTable.getDoubleArrayTopic("robot_orientation_set").publish();
    
    statsSub = cameraTable.getDoubleArrayTopic("hw").subscribe(new double[4]);
    heartbeatSub = cameraTable.getDoubleTopic("hb").subscribe(-1.0);

    // Enforce Pipeline
    cameraTable.getIntegerTopic("pipeline").publish().set(0);

    // Init logging
    String logName = String.format("Limelight/%s/", name);

    fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
    cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpu_temp", "f");
    ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram", "%");
    tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");
    frameCountLog = new IntegerLogEntry(DataLogManager.getLog(), logName + "frame_count");

    // Limelight MegaTag pose listener
    NetworkTableInstance.getDefault().addListener(
      poseSub, 
      EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
      this::processPose
    ); 
  }

  /***
   * Runs when NetworkTables detects a change on the robot pose entry.
   * @param event
   */
  private void processPose(NetworkTableEvent event) {
    // Check that the NetworkTable value is correct
    if (!event.valueData.value.isDoubleArray()) {
      DriverStation.reportError(
        String.format(
          "Robot pose from limelight %s wasn't double[], was %s", 
          cameraName,
          event.valueData.value.getType().getValueStr()
        ),
        false
      );
      return;
    }

    double[] data = event.valueData.value.getDoubleArray();
    double[] stdDevsUpdate = stdDevSub.get();

    // Check that lengths are correct
    if (data.length != 11) {
      DriverStation.reportError(
        String.format("Limelight %s received invalid pose length (expected 11, was %d)", cameraName, data.length),
        false
      );
      return;
    } else if (stdDevsUpdate.length != 12) {
      DriverStation.reportError(
        String.format("Limelight %s received invalid std devs length (expected 12, was %d)", cameraName, stdDevsUpdate.length),
        false
      );
      return;
    }

    // Calculate new Pose
    Pose2d receivedPose = new Pose2d(
      new Translation2d(data[0], data[1]),
      Rotation2d.fromDegrees(data[5])
    );
    double timestamp = event.valueData.value.getTime() - data[6];
    
    // Update standard deviations
    stdDevs.set(0, 0, stdDevsUpdate[0]); // X
    stdDevs.set(1, 0, stdDevsUpdate[1]); // Y
    stdDevs.set(2, 0, stdDevsUpdate[2]); // Yaw

    odometry.addVisionMeasurement(receivedPose, timestamp, stdDevs);
    frameCount++;
  }

  /**
   * Sends the robot orientation to the  limelight.
   */
  public void sendOrientation() {
    orientation[0] = odometry.getFieldRelativePosition().getRotation().getDegrees();
    orientation[1] = odometry.getYawRate();
    // pitch, pitch rate, roll, and roll rate can remain 0.0 indefinitely

    orientationPub.set(orientation);
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

    frameCountLog.append(frameCount);
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

    notConnectedAlert.set(alive);
  }

  /**
   * @return If the limelight is returning a heartbeat.
   */
  public boolean isAlive() {
    return alive;
  }
}
