// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.dashboard.AutoChooser;
import frc.robot.util.dashboard.Dashboard;

public final class RobotContainer {
  // Controllers
  private final CommandXboxController baseController = new CommandXboxController(0);
  private final CommandXboxController armsController = new CommandXboxController(1);

  // Subsystems
  private final SwerveDrive base = new SwerveDrive(baseController::setRumble);
  private final Manipulator manipulator = new Manipulator();
  private final Elevator elevator = new Elevator(manipulator);
  private final Climber climber = new Climber();

  // Dashboard
  private final AutoChooser auto = new AutoChooser(base, elevator);
  private final Dashboard dashboard = new Dashboard(baseController, armsController);
  private final Command pregameCommand = new PregameCommand(base, dashboard, auto);

  public RobotContainer() {
    // Add Pregame command to the dashboard
    SmartDashboard.putData("Pregame", pregameCommand);

    configureBindings();
  }

  private void configureBindings() {
    // Game mode triggers
    Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);

    // SwerveDrive default command (teleop driving)
    base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        () -> -baseController.getLeftX(), 
        () -> -baseController.getLeftY(), 
        () -> -baseController.getRightX(), 
        baseController.leftTrigger())
    );

    // Match timer start/stop
    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());
  }

  /** Checks all hardware, called periodically */
  public void checkHardware() {
    base.checkHardware();
    manipulator.checkHardware();
    elevator.checkHardware();
    climber.checkHardware();
    dashboard.checkHardware();
  }

  /** Logs everything, called periodically */
  public void log() {
    base.log();
    dashboard.update();
    manipulator.log();
    elevator.log();
    climber.log();
  }

  /** Returns the PregameCommand, which is scheduled if the command wasn't run before teleopInit() */
  public Command getPregameCommand() {
    return pregameCommand;
  }

  /** Returns a Command to run in autonomous */
  public Command getAutonomousCommand() {
    return auto.getCurrentSelectedCommand();
  }
}