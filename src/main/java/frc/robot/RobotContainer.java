// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;
import frc.robot.util.Dashboard;

public final class RobotContainer {
  // Subsystems
  private final SwerveDrive base = new SwerveDrive();

  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);

  // Dashboard
  private final AutoChooser auto = new AutoChooser(base);
  private final Dashboard dashboard = new Dashboard();

  public RobotContainer() {
    // Add Pregame command to the dashboard
    SmartDashboard.putData("Pregame", getPregameCommand());

    configureBindings();
  }

  private void configureBindings() {
    // Game mode triggers
    Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);

    // SwerveDrive default command (teleop driving)
    base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        () -> -controller.getLeftX(), 
        () -> -controller.getLeftY(), 
        () -> -controller.getRightX(), 
        controller.leftTrigger())
    );

    // Test go to point command
    controller.a().whileTrue(SwervePaths.getGoToPointCommand(base, new Pose2d(4, 3, Rotation2d.kZero)));

    // Match timer start/stop
    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());
  }

  /** Checks all hardware, called periodically */
  public void checkHardware() {
    base.checkHardware();
  }

  /** Logs everything, called periodically */
  public void log() {
    base.log();
    dashboard.update();
  }

  /** Returns a PregameCommand, which is scheduled if the command wasn't run before teleopInit() */
  public Command getPregameCommand() {
    return new PregameCommand(base, dashboard, auto);
  }

  /** Returns a Command to run in autonomous */
  public Command getAutonomousCommand() {
    return auto.getCurrentSelectedCommand();
  }
}