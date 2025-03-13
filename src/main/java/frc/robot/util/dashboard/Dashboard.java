package frc.robot.util.dashboard;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants.DashboardConstants;

/**
 * Handles some auxiliary Elastic utilities, and a match timer that counts down during real matches
 * and counts up during practice.
 */
public class Dashboard {
  private final DoublePublisher matchTimePub = 
    NetworkTableInstance.getDefault().getDoubleTopic("Elastic/matchTime").publish();
  private final Timer matchTimer = new Timer();

  private final CommandGenericHID[] controllers;
  private final Alert[] controllerAlerts;

  /**
   * @param controllers Controllers to check
   */
  public Dashboard(CommandGenericHID... controllers) {
    this.controllers = controllers;
    controllerAlerts = new Alert[controllers.length];

    for (int c = 0; c < controllers.length; c++) {
      controllerAlerts[c] = new Alert(String.format("Controller %d not connected", c), AlertType.kError);
    }

    // Host Elastic configuration file
    WebServer.start(DashboardConstants.ELASTIC_SERVER_PORT, Filesystem.getDeployDirectory().toPath().resolve("Elastic").toString());

    // Stop the timer (the robot is disabled by default)
    matchTimer.stop();
    matchTimer.reset();

    // Begin hosting CommandScheduler for debugging
    SmartDashboard.putData("Commands", CommandScheduler.getInstance());
  }

  /**
   * Stops the WebServer hosting Elastic's configuration file if the FMS is attached.
   */
  public void stopWebServerIfFMS() {
    if (DriverStation.isFMSAttached()) {
      System.out.println("Stopped Elastic's WebServer because FMS is attached");
      WebServer.stop(DashboardConstants.ELASTIC_SERVER_PORT);
    }
  }

  /**
   * @return A command to restart the timer. Run when the robot is enabled.
   */
  public Command getRestartTimerCommand() {
    return new InstantCommand(matchTimer::restart).ignoringDisable(true);
  }

  /**
   * @return A command to stop the timer. Run when the robot is disabled.
   */
  public Command getStopTimerCommand() {
    return new InstantCommand(matchTimer::stop).ignoringDisable(true);
  }

  /**
   * Sets the rumble to all controllers.
   * @param type
   * @param value
   */
  public void setAllRumble(RumbleType type, double value) {
    for (CommandGenericHID controller : controllers) {
      controller.setRumble(type, value);
    }
  }
  
  public void update() {
    double mt = Timer.getMatchTime();

    if (mt == -1 && !DriverStation.isFMSAttached()) {
      // No FMS match time, count up instead of down
      matchTimePub.set(matchTimer.get());
    } else {
      // Use real match time
      matchTimePub.set(mt);
    }    
  }

  public void checkHardware() {
    for (int c = 0; c < controllers.length; c++) {
      controllerAlerts[c].set(!controllers[c].isConnected());
    }
  }
}
