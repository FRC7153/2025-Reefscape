package frc.robot.util;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;


public class Limelight {
    private String name;

    // Network tables
    private DoubleArraySubscriber poseSub, statsSub;
    private DoubleSubscriber heartbeatSub, txSub;
    private IntegerSubscriber tagInViewSub;
    private DoublePublisher distOut, yawOut;
    private IntegerPublisher priorityTagOut;

    // Cache
    private Translation3d poseCache = new Translation3d();
    private double txAngleCache = 0.0;
    private int tagIdCache = -1;

    // Heartbeat Cache 
    private double lastHeartbeat = -1.0;
    private double lastHeartbeatTS = 0.0;
    private boolean alive = false; 

    // Logging
    private DoubleLogEntry fpsLog, cpuTempLog, ramLog, tempLog, distLog, tagYawLog; 
    private IntegerLogEntry tagIdLog;

    
    /**
     * @param name Host Camera ID
     */
    public Limelight(String name){
        this.name = name;

        // Network Table
        NetworkTable camera = NetworkTableInstance.getDefault().getTable(name);
        NetworkTable out = NetworkTableInstance.getDefault().getTable(String.format("%s-out", name));

        poseSub = 
            camera.getDoubleArrayTopic("targetpose_robotspace").subscribe(new double[6]);

        txSub = 
            camera.getDoubleTopic("tx").subscribe(0.0);

        statsSub = 
            camera.getDoubleArrayTopic("stats").subscribe(new double[4]);
        
        heartbeatSub = 
            camera.getDoubleTopic("heartbeat").subscribe(-1.0);

        tagInViewSub = 
            camera.getIntegerTopic("tagInView").subscribe(-1);
        
        distOut =
            out.getDoubleTopic("dist").publish();
        
        yawOut =
          out.getDoubleTopic("yaw").publish();

        // Enforce Pipeline
        camera.getIntegerTopic("pipeline").publish().set(1);
        
        // Init logging
        String logName = String.format("Hardware/Limelight -%s/", name);

        fpsLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "fps");
        cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "cpu temp", "f" );
        ramLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "ram log");
        tempLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "temp", "f");
        distLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "distance", "m");
        tagYawLog = new DoubleLogEntry(DataLogManager.getLog(), logName + "tag yaw", "deg");
        tagIdLog = new IntegerLogEntry(DataLogManager.getLog(), logName + "tag id");
    }

    /**
     * Caches the latest results, checks the heartbeat, and logs stats.
     * Called periodically
     */
    public void refresh(){
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
    } else if (Timer.getFPGATimestamp() - lastHeartbeatTS <= 0.75) {
      // Recent heartbeat
      alive = true;
    } else {
      // No recent or new heartbeats
      alive = false;
    }

    // Get stats
    double[] stats = statsSub.get();

    if (stats.length < 4) {
      DriverStation.reportWarning("Invalid limelight stats length", false);
    } else {
      fpsLog.append(stats[0]);
      cpuTempLog.append(stats[1]);
      ramLog.append(stats[2]);
      tempLog.append(stats[3]);
    }

    // Get pose
    tagIdCache = (int)tagInViewSub.get();

    if (tagIdCache != -1) {
      double[] poseData = poseSub.get();
      poseCache = new Translation3d(
        poseData[0], 
        poseData[1], 
        poseData[2]
      );

      txAngleCache = txSub.get();
    }

    // Log and output
    distLog.append(getDistanceToTag());
    tagIdLog.append(tagIdCache);
    tagYawLog.append(getTagYaw());

    }

    /**
     * @return Distance, in meters, to the primary in-view tag.
     */
    public double getDistanceToTag(){
      return poseCache.getNorm();
    }
    /**
     * 
     * @return The yaw (deg) tp the primary in-view tag
     */
    public double getTagYaw(){
      return txAngleCache;
    }

    /**
    * @return The fiducial id of the primary tag in-view, or -1 if none.
    */
    public int getTagId() {
      return tagIdCache;
    }

    /**
    * @return If the limelight is returning a heartbeat.
    */
    public boolean isAlive() {
      return alive;
    }

}