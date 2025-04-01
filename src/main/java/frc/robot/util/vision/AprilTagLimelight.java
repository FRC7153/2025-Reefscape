package frc.robot.util.vision;

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
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.swerve.SwerveOdometry;
import frc.robot.util.Util;
import frc.robot.util.logging.ConsoleLogger;

/**
 * @see https://docs.limelightvision.io/docs/docs-limelight/apis/complete-networktables-api
 */
public class AprilTagLimelight extends Limelight {
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

  private final SwerveOdometry odometry;

  // Network tables
  private final DoubleArraySubscriber mt1PoseSub, mt2PoseSub, targetPoseSub, stdDevSub;
  private final DoubleArrayPublisher orientationPub, tagFilterPub;
  private final DoublePublisher imuModePub;
  private final IntegerPublisher throttlePub;

  // Stats
  private int frameCount = 0;

  // Logging
  private final IntegerLogEntry frameCountLog;
  private final BooleanLogEntry isMegaTag2Log;
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
  public AprilTagLimelight(String name, Version version, SwerveOdometry odometry) {
    super(name, version);

    this.odometry = odometry;

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

    throttlePub = cameraTable.getIntegerTopic("throttle_set").publish();
    throttlePub.set(version.hasFans ? 0 : LimelightConstants.DISABLED_THROTTLE);
    
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
      isMegaTag2Pub = null;
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

    if (BuildConstants.PUBLISH_EVERYTHING) {
      positionPub.set(receivedPose);
    }

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

    //if (version.integratedIMU) imuModePub.set(2.0);
  }

  /**
   * @param tags AprilTag Ids to filter for.
   */
  public void setTagIdFilter(double[] tags) {
    tagFilterPub.set(tags);
  }

  /**
   * @param throttle Number of frames to skip between processed frames.
   */
  public void setThrottle(int throttle) {
    throttlePub.set(throttle);
  }

  /**
   * Logs diagnostics data
   */
  @Override
  public void log() {
    super.log();

    frameCountLog.append(frameCount);

    // Log seen fiducial
    seenTagsLog.append(seenTags);
    if (BuildConstants.PUBLISH_EVERYTHING) seenTagsPub.set(seenTags);
  }
}
