package frc.robot.util.dashboard;

import java.nio.ByteBuffer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendableDriveController extends PPHolonomicDriveController implements Sendable {
  private PPHolonomicDriveController controller;
  private PIDConstants translation, rotation;
  private Pose2d currentPose, setpoint;

  public SendableDriveController(PIDConstants translation, PIDConstants rotation) {
    super(translation, rotation);
    this.translation = translation;
    this.rotation = rotation;

    refreshController();
  }

  @Override
  public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
    this.currentPose = currentPose;
    this.setpoint = targetState.pose;
    return controller.calculateRobotRelativeSpeeds(currentPose, targetState);
  }

  private void refreshController() {
    controller = new PPHolonomicDriveController(translation, rotation);
  }

  private void initPIDProperties(SendableBuilder builder, String name, Supplier<PIDConstants> getter, Consumer<PIDConstants> setter) {
    builder.addDoubleProperty(name + " P", () -> getter.get().kP, (double p) -> {
      PIDConstants pid = getter.get();
      setter.accept(new PIDConstants(p, pid.kI, pid.kD, pid.iZone));
      refreshController();
    });
    builder.addDoubleProperty(name + " I", () -> getter.get().kI, (double i) -> {
      PIDConstants pid = getter.get();
      setter.accept(new PIDConstants(pid.kP, i, pid.kD, pid.iZone));
      refreshController();
    });
    builder.addDoubleProperty(name + " D", () -> getter.get().kI, (double d) -> {
      PIDConstants pid = getter.get();
      setter.accept(new PIDConstants(pid.kP, pid.kI, d, pid.iZone));
      refreshController();
    });
  }

  private void initPose2d(SendableBuilder builder, String name, Supplier<Pose2d> getter) {
    ByteBuffer buffer = ByteBuffer.allocate(Pose2d.struct.getSize());

    builder.addRawProperty(
      name, 
      "Pose2d", 
      () -> {
        Pose2d p = getter.get();
        Pose2d.struct.pack(buffer.position(0), p != null ? p : Pose2d.kZero);
        return buffer.array();
      }, 
      null
    );
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    initPIDProperties(builder, "Translation", () -> translation, (PIDConstants t) -> translation = t);
    initPIDProperties(builder, "Rotation", () -> rotation, (PIDConstants r) -> rotation = r);

    initPose2d(builder, "Setpoint", () -> setpoint);
    initPose2d(builder, "Current Pose", () -> currentPose);
  }
}
