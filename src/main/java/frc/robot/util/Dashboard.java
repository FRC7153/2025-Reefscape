package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;
import frc.robot.commands.ResetOdometryToDefaultCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

public final class Dashboard {
  public Dashboard(RobotContainer container, AutoChooser autoChooser, SwerveDrive swerve) {
    // Drive Tab
    ShuffleboardTab drive = Shuffleboard.getTab("Drive");

    // Pregame command button
    drive.add("Pregame", container.getPregameCommand())
      .withPosition(0, 0)
      .withSize(1, 1)
      .withWidget(BuiltInWidgets.kCommand);
    
    // Auto chooser
    drive.add("Auto", autoChooser.getSendableChooser())
      .withPosition(1, 0)
      .withSize(2, 1)
      .withWidget(BuiltInWidgets.kComboBoxChooser);

    // Reset position button
    drive.add("Reset Position", new ResetOdometryToDefaultCommand(swerve))
      .withPosition(0, 1)
      .withSize(1, 1)
      .withWidget(BuiltInWidgets.kCommand);
  }

  public void refresh() {
    // TODO output
  }
}