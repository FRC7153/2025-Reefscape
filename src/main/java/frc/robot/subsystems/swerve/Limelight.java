package frc.robot.subsystems.swerve;

import java.util.EnumSet;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.BuildConstants;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;
import libs.LimelightHelpers;

/**
 * @see https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public class Limelight {
  public static enum Version {
    LIMELIGHT_2PLUS(false),
    LIMELIGHT_3G(false),
    LIMELIGHT_4(true);

    /** Whether this version of limelight has an integrated IMU */
    public final boolean integratedIMU;
    private Version(boolean integratedIMU) {
      this.integratedIMU = integratedIMU;
    }
  }

  // Distance (m) to switch from MT2 to MT1
  private static final double MT2_MIN_DISTANCE = -1.0; // 1.5, -1 to disable MT1

  // Shared orientation array for MegaTag2
  private static final double[] orientation = new double[6];

  /**
   * Sets the shared orientation array output. Pitch, pitch rate, roll, and roll rate will remain
   * 0.0.
   * @param yaw degrees, CCW+
   * @param yawRate degrees/second
   */
  public static void setOrientation(double yaw, double yawRate) {
    orientation[0] = yaw;
    orientation[1] = yawRate;
  }

  // Shared empty translation2d array, used for logging if a limelight sees no tags
  private static final Translation2d[] EMPTY_TRANSLATION_ARRAY = new Translation2d[0];

  private final String cameraName;
  private final Version version;
  private final SwerveOdometry odometry;

  // Network tables
  private final DoubleArraySubscriber mt1PoseSub, mt2PoseSub, targetPoseSub, statsSub, stdDevSub;
  private final DoubleSubscriber heartbeatSub;
  private final DoubleArrayPublisher orientationPub, tagFilterPub;
  private final DoublePublisher imuModePub;
  private final Alert notConnectedAlert;

  // Stats
  private double lastHeartbeat = -1.0;
  private double lastHeartbeatTS = 0.0;
  private boolean alive = false;
  private int frameCount = 0;

  // Logging
  private final DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog;
  private final IntegerLogEntry frameCountLog;
  private final BooleanLogEntry isAliveLog, isMegaTag2Log;
  private final StructArrayLogEntry<Translation2d> seenTagsLog;

  // NT Logging
  private final StructArrayPublisher<Translation2d> seenTagsPub;
  private final StructPublisher<Pose2d> positionPub;
  private final BooleanPublisher isMegaTag2Pub;

  // Cached values
  private final Matrix<N3, N1> stdDevs = VecBuilder.fill(0, 0, 99999); // NEVER trust yaw measurement
  private Translation2d[] seenTags = new Translation2d[0];

  /**
   * @param name Host Camera ID
   */
  public Limelight(String name, Version version, SwerveOdometry odometry) {
    cameraName = name;
    this.version = version;
    this.odometry = odometry;
    notConnectedAlert = new Alert(String.format("Limelight %s is not alive", name), AlertType.kWarning);

    // Get NT topics
    NetworkTable cameraTable = NetworkTableInstance.getDefault().getTable(cameraName);

    mt1PoseSub = cameraTable
      .getDoubleArrayTopic("botpose_wpiblue")
      .subscribe(new double[0]);

    mt2PoseSub = cameraTable
      .getDoubleArrayTopic("botpose_orb_wpiblue")
      .subscribe(new double[0]);

    targetPoseSub = cameraTable
      .getDoubleArrayTopic("targetpose_robotspace")
      .subscribe(new double[0]);

    tagFilterPub = cameraTable.getDoubleArrayTopic("fiducial_id_filters_set").publish();

    stdDevSub = cameraTable.getDoubleArrayTopic("stddevs").subscribe(new double[0]);
    orientationPub = cameraTable.getDoubleArrayTopic("robot_orientation_set").publish();
    
    statsSub = cameraTable.getDoubleArrayTopic("hw").subscribe(new double[4]);
    heartbeatSub = cameraTable.getDoubleTopic("hb").subscribe(-1.0);

    if (version.integratedIMU) {
      imuModePub = cameraTable.getDoubleTopic("imumode_set").publish();
      imuModePub.set(1.0);
    } else {
      imuModePub = null;
    }

    // Enforce Pipeline
    cameraTable.getDoubleTopic("pipeline").publish().set(0);

    // Init logging
    String logName = String.format("Limelight/%s/", name);

    fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
    cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpu_temp", "f");
    ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram", "%");
    tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");
    isAliveLog = new BooleanLogEntry(DataLogManager.getLog(), logName + "is_alive");
    frameCountLog = new IntegerLogEntry(DataLogManager.getLog(), logName + "frame_count");
    isMegaTag2Log = new BooleanLogEntry(DataLogManager.getLog(), logName + "is_megatag_2");
    
    seenTagsLog = StructArrayLogEntry.create(DataLogManager.getLog(), "Limelight/AllTags/" + name, Translation2d.struct);

    // Init NT logging
    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("Limelight/AllTags");
      seenTagsPub = nt.getStructArrayTopic(name, Translation2d.struct).publish();

      NetworkTable nt2 = NetworkTableInstance.getDefault().getTable("Limelight/Poses");
      positionPub = nt2.getStructTopic(name, Pose2d.struct).publish();

      NetworkTable nt3 = NetworkTableInstance.getDefault().getTable("Limelight/" + name);
      isMegaTag2Pub = nt3.getBooleanTopic("IsMegaTag2").publish();
    } else {
      seenTagsPub = null;
      positionPub = null;
    }

    // Limelight MegaTag2 pose listener
    NetworkTableInstance.getDefault().addListener(
      mt2PoseSub, 
      EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
      (NetworkTableEvent event) -> processPose(event, true)
    ); 

    // Limelight MegaTag1 pose listener
    NetworkTableInstance.getDefault().addListener(
      mt1PoseSub, 
      EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
      (NetworkTableEvent event) -> processPose(event, false)
    ); 
  }

  /***
   * Runs when NT detects a change on the robot pose entry.
   * @param event
   */
  private void processPose(NetworkTableEvent event, boolean megaTag2) {
    double[] data = event.valueData.value.getDoubleArray();
    double[] stdDevsUpdate = stdDevSub.get();

    // Check that lengths are correct
    if (data.length < 11 || (data.length != 11 + (int)(data[7] * 7))) {
      ConsoleLogger.reportError(
        String.format("Limelight %s received invalid pose length (was %d)", cameraName, data.length)
      );
      return;
    } else if (stdDevsUpdate.length != 12) {
      ConsoleLogger.reportError(
        String.format("Limelight %s received invalid std devs length (expected 12, was %d)", cameraName, stdDevsUpdate.length)
      );
      return;
    }

    if (data[7] == 0) {
      // No tags here
      seenTags = EMPTY_TRANSLATION_ARRAY;
      return;
    }

    // Get distance to nearest tag
    // If farther than MT2_MIN_DISTANCE, use MegaTag2. Else, use MegaTag1.
    double[] targetPose = targetPoseSub.get();

    if (targetPose.length != 6) {
      ConsoleLogger.reportError(
        String.format("Limelight %s received invalid targetpose length (expected 6, was %d)", cameraName, targetPose.length)
      );

      if (!megaTag2) return;
    } else {
      // For some reason, we need X and Z position here?
      double dist = Math.hypot(targetPose[0], targetPose[2]);
      if ((megaTag2 && dist < MT2_MIN_DISTANCE) || (!megaTag2 && dist >= MT2_MIN_DISTANCE)) return;
    }

    isMegaTag2Log.append(megaTag2);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      isMegaTag2Pub.set(megaTag2);
    }

    // Calculate new Pose
    Translation2d receivedTranslation = new Translation2d(data[0], data[1]);
    Pose2d receivedPose = new Pose2d(receivedTranslation, Rotation2d.fromDegrees(data[5]));

    double timestamp = (event.valueData.value.getTime() / 1000000.0) - (data[6] / 1000.0); // Seconds
    
    // Update standard deviations
    stdDevs.set(0, 0, stdDevsUpdate[megaTag2 ? 6 : 0]); // X
    stdDevs.set(1, 0, stdDevsUpdate[megaTag2 ? 7 : 1]); // Y
    //stdDevs.set(2, 0, stdDevsUpdate[megaTag2 ? 11 : 5]); // Yaw

    odometry.addVisionMeasurement(receivedPose, timestamp, stdDevs);
    frameCount++;
    positionPub.set(receivedPose);

    // Update seen tags array
    int numTags = (int)data[7];
    Translation2d[] seenTagsTemp = new Translation2d[numTags];

    for (int t = 0; t < numTags; t++) {
      seenTagsTemp[t] = Util.getTagPose((int)data[11 + (t * 7)]);
    }

    seenTags = seenTagsTemp;
  }

  /**
   * Sends the robot orientation to the  limelight. Call periodically, after 
   * {@code Limelight.setOrientation(...)}.
   */
  public void sendOrientation() {
    if (version.integratedIMU) imuModePub.set(1.0);

    orientationPub.set(orientation);

    if (version.integratedIMU) imuModePub.set(2.0);
  }

  /**
   * @param tags AprilTag Ids to filter for.
   */
  public void setTagIdFilter(double[] tags) {
    tagFilterPub.set(tags);
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

    // Log seen fiducial
    seenTagsLog.append(seenTags);
    if (BuildConstants.PUBLISH_EVERYTHING) seenTagsPub.set(seenTags);
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

    notConnectedAlert.set(!alive);
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
