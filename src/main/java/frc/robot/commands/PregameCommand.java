package frc.robot.commands;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.dashboard.AutoChooser;
import frc.robot.util.dashboard.Dashboard;

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

  public PregameCommand(SwerveDrive drive, Elevator elevator, Climber climber, Dashboard dashboard, AutoChooser chooser) {
    super(() -> {
      // Run pregame actions:
      drive.pregame();
      elevator.home();

      // Reset encoders
      elevator.resetElevatorEncoder();

      dashboard.stopWebServerIfFMS();

      // Only run auto pregame actions if we are not already in teleop
      if (!DriverStation.isTeleopEnabled()) {
        //FollowPathCommand.warmupCommand().schedule();
        chooser.loadAutoCommand();
      } else {
        System.out.println("Did not pregame auto because teleop is already enabled!");
      }

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