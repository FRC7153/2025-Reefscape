package frc.robot.util;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.RobotContainer;

public final class Dashboard {
    public Dashboard(RobotContainer container, AutoChooser autoChooser) {
        // Drive Tab
        ShuffleboardTab drive = Shuffleboard.getTab("Drive");

        // Pregame command button
        drive.add(container.getPregameCommand())
            .withPosition(0, 0)
            .withSize(1, 1);
        
        // Auto chooser
        drive.add(autoChooser.chooser)
            .withPosition(1, 0)
            .withSize(2, 1);
    }

    public void refresh() {
        // TODO output
    }
}