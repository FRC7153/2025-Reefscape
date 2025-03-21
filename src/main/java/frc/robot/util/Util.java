package frc.robot.util;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.BuildConstants;
import frc.robot.util.logging.ConsoleLogger;
import libs.Elastic;
import libs.Elastic.Notification;
import libs.Elastic.Notification.NotificationLevel;

public class Util {
  /**
   * @return The current stack trace, as a String.
   */
  public static String getCurrentStackTrace() {
    StringBuilder trace = new StringBuilder();

    for (StackTraceElement s : Thread.currentThread().getStackTrace()) {
      trace.append(s.toString());
      trace.append(", ");
    }

    return trace.toString();
  }

  /**
   * @param warnIfFails If a warning should be printed if this fails.
   * @return If the current alliance is red alliance
   */
  public static boolean isRedAlliance(boolean warnIfFails) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    if (alliance.isEmpty()) {
      if (warnIfFails) {
        // Attempted to check Alliance before it was received!
        String trace = getCurrentStackTrace();
        ConsoleLogger.reportError("Invalid alliance received in " + trace);
        
        Elastic.sendNotification(new Notification(
          NotificationLevel.WARNING,
          "Invalid alliance received from FMS",
          trace
        ));
      }
    } else if (alliance.get().equals(Alliance.Red)) {
      // Red alliance confirmed
      return true;
    }

    // Either blue alliance or default
    return false;
  }

  /**
   * @return If the current alliance is red alliance
   */
  public static boolean isRedAlliance() {
    return isRedAlliance(true);
  }

  /**
   * @param value Input value.
   * @param min All inputs less than this are rounded down to 0.
   * @return 0 if value is less than min, else value
   */
  public static double applyDeadband(double value, double min) {
    return (Math.abs(value) < min) ? 0 : value;
  }

  /**
   * @param speeds Input speeds. Modified in place.
   * @param xyMin The translational deadband (m/s).
   * @param omegaMin The rotational deadband (rad/sec).
   */
  public static void deadbandChassisSpeeds(ChassisSpeeds speeds, double xyMin, double omegaMin) {
    speeds.vxMetersPerSecond = applyDeadband(speeds.vxMetersPerSecond, xyMin);
    speeds.vyMetersPerSecond = applyDeadband(speeds.vyMetersPerSecond, xyMin);
    speeds.omegaRadiansPerSecond = applyDeadband(speeds.omegaRadiansPerSecond, omegaMin);
  }

  /**
   * @param tag Tag id
   * @return The 2d position of the tag on the field
   */
  public static Translation2d getTagPose(int tag) {
    Optional<Pose3d> pose = BuildConstants.FIELD.getTagPose(tag);

    if (pose.isEmpty()) {
      // Unknown tag
      ConsoleLogger.reportWarning(String.format("Unknown tag request (id %d)", tag));
      return Translation2d.kZero;
    } else {
      return new Translation2d(pose.get().getX(), pose.get().getY());
    }
  }

  /**
   * Times and outputs the amount of time taken to instantiate an object.
   * @param <T> The type of the object to be instantiated.
   * @param constructor The constructor that instantiates the object.
   * @return The instantiated object
   */
  public static <T> T timeInstantiation(Supplier<T> constructor) {
    double start = Timer.getFPGATimestamp();
    T obj = constructor.get();
    double elapsed = Timer.getFPGATimestamp() - start;

    System.out.printf(
      "%s took %.4f seconds to instantiate\n", obj.getClass().getSimpleName(), elapsed);
    return obj;
  }

  /**
   * @param command The command that may be repeated.
   * @param repeated Whether the command is repeated.
   * @return command.repeatedly if repeated, else just command.
   */
  public static Command possiblyRepeatedCommand(Command command, boolean repeated) {
    return repeated ? command.repeatedly() : command;
  }

  /** Prevent instantiation */
  private Util() {}
}