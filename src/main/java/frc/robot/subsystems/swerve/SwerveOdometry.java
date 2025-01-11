package frc.robot.subsystems.swerve;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.Matrix;
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

/**
 * 4-module SwerveOdometry thread based off of CTRE's SwerveBase.
 */
public final class SwerveOdometry {
    private static final int UPDATE_FREQ = 250;

    private final Thread thread;
    private final ReadWriteLock stateLock = new ReentrantReadWriteLock();
    private final AtomicInteger successfulDAQs = new AtomicInteger();
    private final AtomicInteger failedDAQs = new AtomicInteger();

    private final ADIS16470_IMU imu;
    private final Alert imuHardwareAlert = new Alert("ADIS16470 IMU is not connected", AlertType.kError);

    private final SwerveModulePosition[] swervePositions;
    private final BaseStatusSignal[] allSignals;
    private final SwerveDrivePoseEstimator poseEstimator;
    private boolean isRedAlliance = false; // Cached later

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
            Pose2d.kZero);
    }

    /** Start the odometry thread. */
    public void start() {
        if (thread.isAlive()) {
            DriverStation.reportError("SwerveOdometry.start() called multiple times", false);
        }

        thread.start();
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
    }

    private void run() {
        // Init update frequency
        BaseStatusSignal.setUpdateFrequencyForAll(UPDATE_FREQ, allSignals);
        Threads.setCurrentThreadPriority(true, 1); // Priority 1

        // Run as fast as possible
        while (true) {
            // Wait up to twice period of update frequency
            StatusCode status = BaseStatusSignal.waitForAll(2.0 / UPDATE_FREQ, allSignals);

            if (status.isOK()) {
                successfulDAQs.incrementAndGet();
            } else {
                failedDAQs.incrementAndGet();
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

        if (isRedAlliance)
            pose = Pose2d.kZero; // TODO invert pose, once field is released

        return pose;
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

    public void cacheAllianceColor() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        isRedAlliance = (alliance.isPresent() && alliance.get().equals(Alliance.Red));
        
        System.out.printf("Odometry thread cached isRedAlliance: %b\n", isRedAlliance);
    }

    /**
     * @param visionRobotPoseMeters Position of robot (center) from vision estimation.
     * @param timestampSeconds The timestamp of the vision estimation.
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
}