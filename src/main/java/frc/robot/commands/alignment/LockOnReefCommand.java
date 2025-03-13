package frc.robot.commands.alignment;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BuildConstants;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;
import frc.robot.util.dashboard.LockOnTelemetry;
import frc.robot.util.logging.ConsoleLogger;
import frc.robot.util.math.AlignmentVector;
import frc.robot.util.math.Triangle2d;

public class LockOnReefCommand extends Command {
  /** Reef setpoints that can be locked on to. */
  public static enum ReefSetpoint {
    /** Predefined LEFT reef vectors */
    LEFT(
      new AlignmentVector("REEF_A", new Translation2d(3.658, 4.185), Rotation2d.fromDegrees(0)),
      new AlignmentVector("REEF_C", new Translation2d(3.931, 3.383), Rotation2d.fromDegrees(60)),
      new AlignmentVector("REEF_E", new Translation2d(4.762, 3.219), Rotation2d.fromDegrees(120)),
      new AlignmentVector("REEF_G", new Translation2d(5.321, 3.856), Rotation2d.fromDegrees(180)),
      new AlignmentVector("REEF_I", new Translation2d(5.047, 4.658), Rotation2d.fromDegrees(240)),
      new AlignmentVector("REEF_K", new Translation2d(4.216, 4.823), Rotation2d.fromDegrees(300))
    ),
    /** Predefined CENTER reef vectors */
    CENTER(
      new AlignmentVector("REEF_1", new Translation2d(3.658, 4.021), Rotation2d.fromDegrees(0)),
      new AlignmentVector("REEF_2", new Translation2d(4.074, 3.301), Rotation2d.fromDegrees(60)),
      new AlignmentVector("REEF_3", new Translation2d(4.905, 3.301), Rotation2d.fromDegrees(120)),
      new AlignmentVector("REEF_4", new Translation2d(5.321, 4.021), Rotation2d.fromDegrees(180)),
      new AlignmentVector("REEF_5", new Translation2d(4.905, 4.74), Rotation2d.fromDegrees(240)),
      new AlignmentVector("REEF_6", new Translation2d(4.074, 4.74), Rotation2d.fromDegrees(300))
    ),
    /** Predefined RIGHT reef vectors */
    RIGHT(
      new AlignmentVector("REEF_B", new Translation2d(3.658, 3.856), Rotation2d.fromDegrees(0)),
      new AlignmentVector("REEF_D", new Translation2d(4.216, 3.219), Rotation2d.fromDegrees(60)),
      new AlignmentVector("REEF_F", new Translation2d(5.047, 3.383), Rotation2d.fromDegrees(120)),
      new AlignmentVector("REEF_H", new Translation2d(5.321, 4.185), Rotation2d.fromDegrees(180)),
      new AlignmentVector("REEF_J", new Translation2d(4.762, 4.823), Rotation2d.fromDegrees(240)),
      new AlignmentVector("REEF_L", new Translation2d(3.931, 4.658), Rotation2d.fromDegrees(300))
    );

    public final AlignmentVector[] vectors;
    private ReefSetpoint(AlignmentVector... vectors) {
      this.vectors = vectors;
    }
  }

  // Really shoddy math
  private static final Translation2d REEF_CENTER = new Translation2d(4.476, 4.493);
  private static final Translation2d[] REEF_CORNERS_EXPANDED = {
    new Translation2d(-8.514, 11.993),
    new Translation2d(-8.514, -3.007),
    new Translation2d(4.476, -10.507),
    new Translation2d(17.467, -3.007),
    new Translation2d(17.467, 11.993),
    new Translation2d(4.476, 19.493)
  };
  private static final Triangle2d[] REEF_ZONES = {
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[0], REEF_CORNERS_EXPANDED[1]), // A/B
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[1], REEF_CORNERS_EXPANDED[2]), // C/D
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[2], REEF_CORNERS_EXPANDED[3]), // E/F
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[3], REEF_CORNERS_EXPANDED[4]), // G/H
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[4], REEF_CORNERS_EXPANDED[5]), // I/J
    new Triangle2d(REEF_CENTER, REEF_CORNERS_EXPANDED[5], REEF_CORNERS_EXPANDED[0]), // K/L
  };

  private final SwerveDrive drive;
  private final Supplier<Double> xSupplier, ySupplier;
  private final BiConsumer<RumbleType, Double> rumbleConsumer;
  private final ReefSetpoint setpoints;
  private final PathPlannerTrajectoryState targetState = new PathPlannerTrajectoryState();

  private AlignmentVector vector;

  /**
   * Locks onto the best Reef target of the ones supplied in setpoints.
   * @param drive
   * @param xSupplier X input supplier (%)
   * @param ySupplier Y input supplier (%)
   * @param rumbleConsumers Consumers for haptic feedback
   * @param setpoints List of setpoints to follow
   */
  public LockOnReefCommand(
    SwerveDrive drive, 
    Supplier<Double> xSupplier,
    Supplier<Double> ySupplier,
    BiConsumer<RumbleType, Double> rumbleConsumers,
    ReefSetpoint setpoints
  ) {
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rumbleConsumer = rumbleConsumers;
    this.setpoints = setpoints;

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Determine which side of the reef to use
    Pose2d currentPose = drive.getPosition(false);
    vector = null;

    for (int i = 0; i < REEF_ZONES.length; i++) {
      if (REEF_ZONES[i].containsPoint(currentPose.getTranslation())) {
        // We are in this zone
        vector = setpoints.vectors[i];
        break;
      }
    }

    // Make sure we were in at least 1 zone
    if (vector == null) {
      ConsoleLogger.reportError("Robot position is not in any reef zone!");
      vector = setpoints.vectors[0];
    }

    // Output
    LockOnTelemetry.setTargetName(vector.getName()); 
  }

  @Override
  public void execute() {
    // Get current pose
    Pose2d currentPose = drive.getPosition(false);
    
    // Get user input (note y and x are swapped here, because forward (y+) should be a vector of 0 degrees)
    Translation2d speed = new Translation2d(
      ySupplier.get() * SwerveConstants.SLOW_TRANSLATIONAL_SPEED,
      xSupplier.get() * SwerveConstants.SLOW_TRANSLATIONAL_SPEED
    );

    double projectedSpeedMagnitude = vector.projectVector(speed).getNorm();

    // Determine target position
    Translation2d projection = vector.projectPoint(
      currentPose.getTranslation(), 
      projectedSpeedMagnitude * TimedRobot.kDefaultPeriod
    );

    // Get drive base speed
    targetState.pose = new Pose2d(projection, vector.getDirection());
    targetState.heading = vector.getDirection();
    targetState.linearVelocity = projectedSpeedMagnitude;

    ChassisSpeeds chassis = 
      SwerveConstants.AUTO_CONTROLLER.calculateRobotRelativeSpeeds(currentPose, targetState);

    Util.deadbandChassisSpeeds(chassis, 0.05, 0.01);
    
    // Drive
    drive.drive(chassis, true);

    // Feedback
    double dist = currentPose.getTranslation().getDistance(vector.getTarget());

    double rumble = (Math.abs(dist) <= BuildConstants.EPSILON) ? 1.0 : Math.min(0.007 / dist, 1.0);
    rumbleConsumer.accept(RumbleType.kRightRumble, rumble);

    LockOnTelemetry.setTargetSetpoint(targetState.pose, dist);
  }

  @Override
  public void end(boolean interrupted) {
    rumbleConsumer.accept(RumbleType.kRightRumble, 0.0);
    LockOnTelemetry.clearTargetName();

    if (interrupted) drive.stop();
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }

  @Override
  public String getName() {
    return String.format("LockOnReefCommand(%s)", setpoints.name());
  }
}
