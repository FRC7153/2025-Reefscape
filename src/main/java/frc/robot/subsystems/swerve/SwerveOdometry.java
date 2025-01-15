package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.CalibrationTime;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;

/**
 * 4-module SwerveOdometry thread based off of CTRE's SwerveBase.
 */
public final class SwerveOdometry {
  private static final int UPDATE_FREQ = 250;

  private final Thread thread;
  private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
  private final AtomicInteger successfulDAQs = new AtomicInteger();
  private final AtomicInteger failedDAQs = new AtomicInteger();
  private final AtomicInteger actualFreq = new AtomicInteger();

  private final ADIS16470_IMU imu;
  private final Alert imuHardwareAlert = new Alert("ADIS16470 IMU is not connected", AlertType.kError);

  private final SwerveModulePosition[] swervePositions;
  private final BaseStatusSignal[] allSignals;
  private final SwerveDrivePoseEstimator poseEstimator;
  private boolean isRedAlliance = false; // Cached later
  private boolean hasSetInitialPosition = false; // Set later

  private double startTime; // For freq calculation

  /**
   * Automatically starts odometry thread.
   * @param modules
   * @param kinematics
   */
  public SwerveOdometry(SwerveModule[] modules, SwerveDriveKinematics kinematics) {        
    // Init thread
    thread = new Thread(this::run);
    thread.setDaemon(true);

    // Init IMU
    imu = new ADIS16470_IMU(
      SwerveConstants.GYRO_YAW,
      SwerveConstants.GYRO_PITCH,
      SwerveConstants.GYRO_ROLL,
      Port.kOnboardCS0,
      CalibrationTime._4s);

    // Get status signal and positions
    allSignals = new BaseStatusSignal[4*2];
    swervePositions = new SwerveModulePosition[4];

    for (int m = 0; m < 4; m++) {
      allSignals[m*2+0] = modules[m].getDrivePosition();
      allSignals[m*2+1] = modules[m].getSteerAngle();

      swervePositions[m] = new SwerveModulePosition(
        allSignals[m*2+0].getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE, 
        Rotation2d.fromRotations(allSignals[m*2+1].getValueAsDouble())
      );
    }

    // Init pose estimator
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics, 
      Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kYaw)), 
      swervePositions, 
      Pose2d.kZero,
      SwerveConstants.STATE_STD_DEVS, // State std devs
      VecBuilder.fill(0.9, 0.9, 0.9) // Vision std devs, will be changed in calls to addVisionMeasurement(...)
    );
  }

  /** Start the odometry thread. */
  public void start() {
    if (thread.isAlive()) {
      DriverStation.reportError("SwerveOdometry.start() called multiple times", false);
    } else {
      startTime = Timer.getFPGATimestamp() + 3.0; // 3 seconds before average frequency is calculated
      thread.start();
    }
  }

  /**
   * @param newPosition New Position to completely reset odometry to.
   */
  public void resetPosition(Pose2d newPosition) {
    try {
      stateLock.writeLock().lock();
      poseEstimator.resetPosition(
        Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kYaw)), 
        swervePositions, // this should be updated in place
        newPosition);
    } finally {
      stateLock.writeLock().unlock();
    }

    hasSetInitialPosition = true;
  }

  /**
   * If no initial position has yet been set, it will set one depending on the cached alliance
   * color.
   */
  public void setDefaultPosition() {
    if (!hasSetInitialPosition) {
      Pose2d initial = isRedAlliance ? SwerveConstants.DEFAULT_RED_POSE : SwerveConstants.DEFAULT_BLUE_POSE;
      System.out.printf(
        "Configured an initial position for %s alliance: %s\n", 
        isRedAlliance ? "RED" : "BLUE", 
        initial);
      resetPosition(initial);
    }
  }

  private void run() {
    // Init update frequency
    BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQ, allSignals);
    Threads.setCurrentThreadPriority(true, 1); // Priority 1

    int totalRuns = 0; // for calculating frequency, not including first 3 seconds

    // Run as fast as possible
    while (true) {
      // Wait up to twice period of update frequency
      StatusCode status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQ, allSignals);

      if (status.isOK()) {
        successfulDAQs.incrementAndGet();
      } else {
        failedDAQs.incrementAndGet();
        continue;
      }

      try {
        stateLock.writeLock().lock();

        // Update swerve module positions
        for (int m = 0; m < 4; m++) {
          swervePositions[m].distanceMeters = 
            allSignals[m*2+0].getValueAsDouble() * SwerveConstants.WHEEL_CIRCUMFERENCE;
          swervePositions[m].angle = 
            Rotation2d.fromRotations(allSignals[m*2+1].getValueAsDouble());
        }

        // Update estimator
        poseEstimator.update(
          Rotation2d.fromDegrees(imu.getAngle(IMUAxis.kYaw)), 
          swervePositions);
      } finally {
        stateLock.writeLock().unlock();
      }

      // Update frequency
      double timeDelta = Timer.getFPGATimestamp() - startTime;

      if (timeDelta >= 0.0) {
        totalRuns++;
        actualFreq.set((int)(totalRuns / timeDelta));
      }

      // Use this to determine which axis is yaw:
      /*System.out.printf(
        "IMU: X: %f, Y: %f, Z: %f\n", 
        imu.getAngle(IMUAxis.kX), imu.getAngle(IMUAxis.kY), imu.getAngle(IMUAxis.kZ));*/
    }
  }

  /**
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#always-blue-origin
   * @return The most recent position estimation, relative to the field.
   */
  public Pose2d getFieldRelativePosition() {
    try {
      stateLock.readLock().lock();
      return poseEstimator.getEstimatedPosition();
    } finally {
      stateLock.readLock().unlock();
    }
  }

  /**
   * @see https://docs.wpilib.org/en/stable/docs/software/basic-programming/coordinate-system.html#robot-drive-kinematics
   * @return The most recent position estimation, relative to the alliance.
   */
  public Pose2d getAllianceRelativePosition() {
    Pose2d pose = getFieldRelativePosition();

    // Flip if red alliance
    return isRedAlliance ? FlippingUtil.flipFieldPose(pose) : pose;
  }

  /**
   * @return The number of successful data acquisitions.
   */
  public int getSuccessfulDAQs() {
    return successfulDAQs.get();
  }

  /**
   * @return The number of failed data acquisitions.
   */
  public int getFailedDAQs() {
    return failedDAQs.get();
  }

  /**
   * @return The approximate frequency of the odometry loop, in hertz (per second).
   */
  public int getFrequency() {
    return actualFreq.get();
  }

  public void cacheAllianceColor() {
    Optional<Alliance> alliance = DriverStation.getAlliance();
    isRedAlliance = (alliance.isPresent() && alliance.get().equals(Alliance.Red));
    
    System.out.printf("Odometry thread cached isRedAlliance: %b\n", isRedAlliance);
  }

  /**
   * @param visionRobotPoseMeters Position of robot (center) from vision estimation.
   * @param timestampSeconds The timestamp of the vision estimation (seconds).
   * @param visionMeasurementStdDevs The standard deviations of the vision estimation.
   */
  public void addVisionMeasurement(
    Pose2d visionRobotPoseMeters, 
    double timestampSeconds, 
    Matrix<N3, N1> visionMeasurementStdDevs) {
    try {
      stateLock.writeLock().lock();
      poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    }  finally {
      stateLock.writeLock().unlock();
    }
  }

  public void checkHardware() {
    imuHardwareAlert.set(!imu.isConnected());
  }

  public double getYawRate(){
   return imu.getRate(IMUAxis.kYaw);
  }
}