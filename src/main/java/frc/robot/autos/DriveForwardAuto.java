package frc.robot.autos;

import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.Util;

public class DriveForwardAuto extends SequentialCommandGroup {
  private final String name;

  /**
   * Drives forward (should be robot backwards) for a certain amount of time.
   * @param drive
   * @param startingPose alliance relative starting position.
   */
  public DriveForwardAuto(SwerveDrive drive, Pose2d startingPose) {
    super(
      // Reset position first
      new InstantCommand(
        () -> drive.resetOdometry(Util.isRedAlliance() ? FlippingUtil.flipFieldPose(startingPose) : startingPose),
        drive
      ),
      // Drive forwards for time
      new InstantCommand(() -> drive.drive(0.0, -1.8, 0.0, true, true), drive),
      new WaitCommand(0.75),
      new InstantCommand(drive::stop, drive),
      // Finished
      new PrintCommand("DriveForwardAuto finished.")
    );

    name = String.format("DriveForwardAuto(%s)", startingPose.toString());
  }

  @Override
  public String getName() {
    return name;
  }
}
