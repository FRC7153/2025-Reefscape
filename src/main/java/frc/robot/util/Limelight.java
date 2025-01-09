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
import frc.robot.Constants.BuildConstants;

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
    private IntegerLogEntry tagIdLong;


    /**
     * 
     * @param name Host Camera ID
     */
    public LimelightCamera(String name){
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
        cpuTempLog = new DoubleLogEntry(DataLogManager.getLog(), logName)
    }
}