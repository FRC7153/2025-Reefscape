package frc.robot.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

/**
 * Runs all actions that should be run after the robot successfully boots/initializes/connects,
 * but BEFORE the match actually starts
 */
public class PregameCommand extends InstantCommand {
    private static boolean hasPregamed = false;
    private static final Alert noPregameAlert = new Alert("Pregame not run yet!", AlertType.kWarning);

    static {
        // By default, this alert should be true:
        noPregameAlert.set(true);
    }

    public static boolean getHasPregamed() { return hasPregamed;  }

    public PregameCommand(SwerveDrive drive) {
        super(() -> {
            // Run pregame actions:
            drive.homeEncoders();
            drive.cacheAllianceColor();

            // Notify:
            System.out.println("Pregame complete");
            hasPregamed = true;
            noPregameAlert.set(false);
        });
    }

    @Override
    public String getName() { return "PREGAME"; }

    @Override
    public boolean runsWhenDisabled() { return true; }
}