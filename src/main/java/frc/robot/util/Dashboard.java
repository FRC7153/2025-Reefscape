package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;

public final class Dashboard {
  public Dashboard(RobotContainer container, AutoChooser autoChooser) {
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
  }

  public void refresh() {
    // TODO output
  }
}