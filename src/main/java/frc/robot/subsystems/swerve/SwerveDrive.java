package frc.robot.subsystems.swerve;

import java.util.List;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.ResetOdometryToDefaultCommand;
import frc.robot.util.logging.ConsoleLogger;

public final class SwerveDrive implements Subsystem {
  // Swerve Modules
  protected final SwerveModule[] modules = {
    new SwerveModule(
        "FL", 
        HardwareConstants.FL_DRIVE_KRAKEN_CAN, HardwareConstants.FL_STEER_NEO_CAN, 
        HardwareConstants.FL_STEER_CANCODER_CAN, SwerveConstants.FL_CANCODER_OFFSET
    ), new SwerveModule(
        "FR", 
        HardwareConstants.FR_DRIVE_KRAKEN_CAN, HardwareConstants.FR_STEER_NEO_CAN, 
        HardwareConstants.FR_STEER_CANCODER_CAN, SwerveConstants.FR_CANCODER_OFFSET
    ), new SwerveModule(
        "RL", 
        HardwareConstants.RL_DRIVE_KRAKEN_CAN, HardwareConstants.RL_STEER_NEO_CAN, 
        HardwareConstants.RL_STEER_CANCODER_CAN, SwerveConstants.RL_CANCODER_OFFSET
    ), new SwerveModule(
        "RR", 
        HardwareConstants.RR_DRIVE_KRAKEN_CAN, HardwareConstants.RR_STEER_NEO_CAN, 
        HardwareConstants.RR_STEER_CANCODER_CAN, SwerveConstants.RR_CANCODER_OFFSET
    )
  };

  // Kinematics
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.POSITIONS);
  private ChassisSpeeds lastChassisSpeeds = new ChassisSpeeds();

  // NT Logging
  private final StructArrayPublisher<SwerveModuleState> statePublisher, reqStatePublisher;
  private final StructPublisher<Pose2d> posePublisher;
  private final IntegerPublisher successfulDAQPublisher, failedDAQPublisher;
  private final DoublePublisher odometryFreqPublisher;
  private final BooleanPublisher isClosedLoopPublisher;
  private final Field2d fieldPublisher = new Field2d();

  // DL Logging
  private final StructArrayLogEntry<SwerveModuleState> stateLogger = 
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/State", SwerveModuleState.struct);
  private final StructArrayLogEntry<SwerveModuleState> reqStateLogger = 
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/Request", SwerveModuleState.struct);
  private final BooleanLogEntry isClosedLoopLogger =
    new BooleanLogEntry(DataLogManager.getLog(), "Swerve/ClosedLoop");
  private final StructLogEntry<Pose2d> poseLogger =
    StructLogEntry.create(DataLogManager.getLog(), "Swerve/Pose", Pose2d.struct);
  private final StructArrayLogEntry<Pose2d> trajectoryLogger =
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/Trajectory", Pose2d.struct);
  private final IntegerLogEntry successfulDAQLogger = 
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Successful_DAQs");
  private final IntegerLogEntry failedDAQLogger =
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Failed_DAQs");
  private final DoubleLogEntry odometryFreqLogger =
    new DoubleLogEntry(DataLogManager.getLog(), "Swerve/Odometry_Freq");

  private final SwerveModuleState[] currentStates = {
    modules[0].state, modules[1].state, modules[2].state, modules[3].state
  };
  private final SwerveModuleState[] currentRequests = new SwerveModuleState[4];

  // Pose estimation
  protected final SwerveOdometry odometry = new SwerveOdometry(modules, kinematics);
  private SysIdRoutine pathRoutine;

  private final Limelight limelightMain = new Limelight("limelight-main", odometry);

  // Autonomous
  protected final RobotConfig autoConfig;
  private final Alert failedToLoadConfigAlert = new Alert("Failed to load PathPlanner's config", AlertType.kError);

  @SuppressWarnings("UseSpecificCatch")
  public SwerveDrive() {
    if (BuildConstants.PUBLISH_EVERYTHING) {
      // Init NT publishers
      NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Swerve");

      statePublisher = ntTable.getStructArrayTopic("State", SwerveModuleState.struct).publish();
      reqStatePublisher = ntTable.getStructArrayTopic("Request", SwerveModuleState.struct).publish();
      posePublisher = ntTable.getStructTopic("Pose", Pose2d.struct).publish();
      successfulDAQPublisher = ntTable.getIntegerTopic("Successful_DAQs").publish();
      failedDAQPublisher = ntTable.getIntegerTopic("Failed_DAQs").publish();
      odometryFreqPublisher = ntTable.getDoubleTopic("Odometry_Freq").publish();
      isClosedLoopPublisher = ntTable.getBooleanTopic("IsClosedLoop").publish();
    } else {
      // Do not init NT publishers
      statePublisher = null;
      reqStatePublisher = null;
      posePublisher = null;
      successfulDAQPublisher = null;
      failedDAQPublisher = null;
      odometryFreqPublisher = null;
      isClosedLoopPublisher = null;
    }

    // Start odometry thread
    odometry.start();

    // Load autonomous config
    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
      System.out.println("Loaded PathPlanner's RobotConfig");
    } catch (Exception e) {
      // Failed to load RobotConfig
      config = null;
      ConsoleLogger.reportWarning(
        String.format("Failed to load PathPlanner's config: %s", e.getMessage()) 
      );
      failedToLoadConfigAlert.set(true);
    }

    autoConfig = config;

    // Add reset position command to dashboard
    SmartDashboard.putData("Reset position", new ResetOdometryToDefaultCommand(this));

    // Add Field2d to dashboard
    SmartDashboard.putData("Field", fieldPublisher);

    // Initialize trajectory logging
    PPLibTelemetry.enableCompetitionMode();

    PathPlannerLogging.setLogActivePathCallback((List<Pose2d> path) -> {
      trajectoryLogger.append(path);
      fieldPublisher.getObject("trajectory").setPoses(path);
    });

    // Warm up PathPlanner
    FollowPathCommand.warmupCommand().schedule();
  }

  /**
   * @param speeds robot-relative ChassisSpeeds
   * @param closedLoop
   */
  public void drive(ChassisSpeeds speeds, boolean closedLoop) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_WHEEL_VELOCITY);

    lastChassisSpeeds = speeds;

    for (int m = 0; m < 4; m++) {
      modules[m].setRequest(states[m], closedLoop);
    }

    isClosedLoopLogger.append(closedLoop);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      isClosedLoopPublisher.set(closedLoop);
    }
  }

  /**
   * @param y Left is +, m/s
   * @param x Fwd is +, m/s
   * @param theta CCW is +, rad/sec
   * @param closedLoop
   * @param fieldOriented
   */
  public void drive(double y, double x, double theta, boolean closedLoop, boolean fieldOriented) {
    drive(
      fieldOriented ? 
        // Field oriented driving
        ChassisSpeeds.fromRobotRelativeSpeeds(x, y, theta, odometry.getAllianceRelativePosition().getRotation().unaryMinus())
        // Robot oriented driving
        : new ChassisSpeeds(x, y, theta), 
      closedLoop
    );
  }

  /**
   * Stops all the motors.
   */
  public void stop() {
    drive(new ChassisSpeeds(), false);
  }

  /** Gets robot-relative chassis speeds */
  protected ChassisSpeeds getCurrentChassisSpeeds() {
    // currentStates is updated in place periodically
    return kinematics.toChassisSpeeds(currentStates);
  }

  /** Resets from a FIELD RELATIVE position */
  public void resetOdometry(Pose2d newPose) {
    Pose2d pose = odometry.getFieldRelativePosition();
    odometry.resetPosition(newPose);
    System.out.printf("Reset odometry from %s -> %s\n", pose, newPose);
  }

  @Override
  public void periodic() {
    // Update current state and current request
    for (int m = 0; m < 4; m++) {
      /*
       * The method below updates the SwerveModuleState objects in the 'loggedStates' array in place
       * and returns the most recently requested state.
       */
      currentRequests[m] = modules[m].getAndUpdateStates();
    }

    // Update limelights
    Limelight.setOrientation(
      odometry.getFieldRelativePosition().getRotation().getDegrees(), odometry.getYawRate());
      
    limelightMain.sendOrientation();
  }

  /** Gets routine for Auto Path SysId characterization. */
  public SysIdRoutine getPathRoutine() {
    if (pathRoutine == null) {
      // No cached routine, instantiate it
      pathRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.75).per(Second), Volts.of(1.25), null), 
        new SysIdRoutine.Mechanism(
          (Voltage v) -> {
            drive(v.in(Volts), 0.0, 0.0, true, false);
          },
          (SysIdRoutineLog log) -> {
            log.motor("base")
              .voltage(Volts.of(lastChassisSpeeds.vyMetersPerSecond))
              .linearPosition(Meters.of(odometry.getFieldRelativePosition().getY()))
              .linearVelocity(MetersPerSecond.of(odometry.getYVelocity()));
          },
          this
        )
      );
    }

    return pathRoutine;
  }

  /**
   * @param fieldRelative Relative to blue alliance if true, else relative to the current alliance.
   * @return Position of robot on field.
   */
  public Pose2d getPosition(boolean fieldRelative) {
    return fieldRelative ? odometry.getFieldRelativePosition() : odometry.getAllianceRelativePosition();
  }

  /** Homes all swerve modules and caches alliance color for odometry. Run in pregame. */
  public void pregame() {
    for (int m = 0; m < 4; m++) {
      modules[m].homeEncoder();
    }

    odometry.cacheAllianceColor();
  }

  public void log() {
    // Get pose
    Pose2d pose = odometry.getFieldRelativePosition();

    // Log
    stateLogger.append(currentStates);
    reqStateLogger.append(currentRequests);
    poseLogger.append(pose);
    successfulDAQLogger.append(odometry.getSuccessfulDAQs());
    failedDAQLogger.append(odometry.getFailedDAQs());
    odometryFreqLogger.append(odometry.getFrequency());

    // Update field2d
    fieldPublisher.getRobotObject().setPose(pose);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      statePublisher.set(currentStates);
      reqStatePublisher.set(currentRequests);
      posePublisher.set(pose);
      successfulDAQPublisher.set(odometry.getSuccessfulDAQs());
      failedDAQPublisher.set(odometry.getFailedDAQs());
      odometryFreqPublisher.set(odometry.getFrequency());
    }

    // Log limelights
    limelightMain.log();
  }

  public void checkHardware() {
    for (int m = 0; m < 4; m++) {
      modules[m].checkHardware();
    }

    odometry.checkHardware();
    limelightMain.checkHardware();
  }
}
