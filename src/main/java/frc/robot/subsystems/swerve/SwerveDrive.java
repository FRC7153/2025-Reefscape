package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.SignalLogger;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.util.datalog.StructArrayLogEntry;
import edu.wpi.first.util.datalog.StructLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;

public final class SwerveDrive implements Subsystem {
  // Swerve Modules
  private final SwerveModule[] modules = {
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
        "FL", 
        HardwareConstants.RR_DRIVE_KRAKEN_CAN, HardwareConstants.RR_STEER_NEO_CAN, 
        HardwareConstants.RR_STEER_CANCODER_CAN, SwerveConstants.RR_CANCODER_OFFSET
    )
  };

  // Kinematics
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SwerveConstants.POSITIONS);

  // NT Logging
  private final StructArrayPublisher<SwerveModuleState> statePublisher, reqStatePublisher;
  private final StructPublisher<Pose2d> posePublisher;
  private final IntegerPublisher successfulDAQPublisher, failedDAQPublisher;
  private final BooleanPublisher isClosedLoopPublisher;

  // DL Logging
  private final StructArrayLogEntry<SwerveModuleState> stateLogger = 
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/State", SwerveModuleState.struct);
  private final StructArrayLogEntry<SwerveModuleState> reqStateLogger = 
    StructArrayLogEntry.create(DataLogManager.getLog(), "Swerve/Request", SwerveModuleState.struct);
  private final BooleanLogEntry isClosedLoopLogger =
    new BooleanLogEntry(DataLogManager.getLog(), "Swerve/ClosedLoop");
  private final StructLogEntry<Pose2d> poseLogger =
    StructLogEntry.create(DataLogManager.getLog(), "Swerve/Pose", Pose2d.struct);
  private final IntegerLogEntry successfulDAQLogger = 
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Successful_DAQs");
  private final IntegerLogEntry failedDAQLogger =
    new IntegerLogEntry(DataLogManager.getLog(), "Swerve/Failed_DAQs");

  private final SwerveModuleState[] currentStates = {
    modules[0].state, modules[1].state, modules[2].state, modules[3].state
  };
  private final SwerveModuleState[] currentRequests = new SwerveModuleState[4];

  // Pose estimation
  private final SwerveOdometry odometry = new SwerveOdometry(modules, kinematics);
  private SysIdRoutine moduleRoutine, pathRoutine;

  // Autonomous
  private final RobotConfig autoConfig;

  public SwerveDrive() {
    if (BuildConstants.PUBLISH_EVERYTHING) {
      // Init NT publishers
      NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Swerve");

      statePublisher = ntTable.getStructArrayTopic("State", SwerveModuleState.struct).publish();
      reqStatePublisher = ntTable.getStructArrayTopic("Request", SwerveModuleState.struct).publish();
      posePublisher = ntTable.getStructTopic("Pose", Pose2d.struct).publish();
      successfulDAQPublisher = ntTable.getIntegerTopic("Successful_DAQs").publish();
      failedDAQPublisher = ntTable.getIntegerTopic("Failed_DAQs").publish();
      isClosedLoopPublisher = ntTable.getBooleanTopic("IsClosedLoop").publish();
    } else {
      // Do not init NT publishers
      statePublisher = null;
      reqStatePublisher = null;
      posePublisher = null;
      successfulDAQPublisher = null;
      failedDAQPublisher = null;
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
      DriverStation.reportWarning(
        String.format("Failed to load PathPlanner's config: %s", e.getMessage()), 
        false
      );
      // TODO error handler
      e.printStackTrace();

      new Alert("Failed to load PathPlanner's RobotConfig", AlertType.kError);
    }

    autoConfig = config;
  }

  /**
   * @param speeds robot-relative ChassisSpeeds
   * @param closedLoop
   */
  public void drive(ChassisSpeeds speeds, boolean closedLoop) {
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.MAX_WHEEL_VELOCITY);

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
    ChassisSpeeds speeds = fieldOriented ?
      // Field oriented driving
      ChassisSpeeds.fromRobotRelativeSpeeds(x, y, theta, odometry.getFieldRelativePosition().getRotation().unaryMinus())
      // Robot oriented driving
      : new ChassisSpeeds(x, y, theta);

    drive(speeds, closedLoop);
  }

  /**
   * @param pathName Name of path to follow
   * @return
   */
  public Command getFollowPathCommand(String pathName, boolean resetPosition) {
    try {
      PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
      Pose2d start = path.getStartingHolonomicPose().get(); // TODO this
      
      return new FollowPathCommand(
        path, 
        odometry::getFieldRelativePosition, 
        this::getCurrentChassisSpeeds, 
        (ChassisSpeeds speeds, DriveFeedforwards feedforwards) -> {
          drive(speeds, true);
        }, 
        SwerveConstants.AUTO_CONTROLLER, 
        autoConfig, 
        () -> false, 
        this
      );
    } catch (Exception e) {
      // TODO error handling
      return new PrintCommand(String.format("Running failed path: '%s'", pathName));
    }
  }

  public ChassisSpeeds getCurrentChassisSpeeds() {
    // currentStates is updated in place periodically
    return kinematics.toChassisSpeeds(currentStates);
  }

  public Pose2d getAllianceRelativePose() {
    return odometry.getAllianceRelativePosition();
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
  }

  /** Gets routine for MODULE SysId characterization. This will start the CTRE SignalLogger */
  public SysIdRoutine getModuleRoutine() {
    System.out.println("Starting CTRE SignalLogger due to getRoutine call in SwerveDrive");
    SignalLogger.start();

    if (moduleRoutine == null) {
      // No cached routine, instantiate it
      moduleRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, (State state) -> {
          // Log state
          SignalLogger.writeString("drive-sysid-state", state.toString());
        }), 
        new SysIdRoutine.Mechanism((Voltage v) -> {
          // Apply voltages
          for (int m = 0; m < 4; m++) {
            modules[m].setVoltageRequest(v.in(Volts));
          }
        }, null, this)
      );
    }

    return moduleRoutine;
  }

  /** Gets routine for Auto Path SysId characterization. */
  public SysIdRoutine getPathRoutine() {
    /*if (pathRoutine == null) {
      // No cached routine, instantiate it
      pathRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(
          (Voltage v) -> {
            drive(0.0, v.in(Volts), 0.0, true, false);
          },
          (SysIdRoutineLog log) -> {
            log.motor("base")
              .voltage(0.0)
              .linearPosition(0.0)
              .linearVelocity(0.0);
          },
          this
        )
      );
    }*/

    return pathRoutine;
  }

  /** Homes all swerve modules. Run in pregame. */
  public void homeEncoders() {
    for (int m = 0; m < 4; m++) {
      modules[m].homeEncoder();
    }
  }

  /** Caches alliance color for odometry. Run in pregame. */
  public void cacheAllianceColor() {
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

    if (BuildConstants.PUBLISH_EVERYTHING) {
      statePublisher.set(currentStates);
      reqStatePublisher.set(currentRequests);
      posePublisher.set(pose);
      successfulDAQPublisher.set(odometry.getSuccessfulDAQs());
      failedDAQPublisher.set(odometry.getFailedDAQs());
    }
  }

  public void checkHardware() {
    for (int m = 0; m < 4; m++) {
      modules[m].checkHardware();
    }
  }
}
