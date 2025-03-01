// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.TestCommand;
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
  private final Command pregameCommand = new PregameCommand(base, elevator, dashboard, auto);

  public RobotContainer() {
    // Add Pregame command to the dashboard
    SmartDashboard.putData("Pregame", pregameCommand);

    configureBindings();
  }

  private void configureBindings() {
    // Triggers
    Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);
    Trigger isTestTrigger = new Trigger(DriverStation::isTestEnabled);
    Trigger isRollLimitExceededTrigger = new Trigger(base::getRollLimitExceeded);

    // SwerveDrive default command (teleop driving)
    /*base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        () -> -baseController.getLeftX(), 
        () -> -baseController.getLeftY(), 
        () -> -baseController.getRightX(), 
        baseController.leftTrigger())
    );*/
    
    // Climber default command (climb if both buttons pressed)
    climber.setDefaultCommand(
      new ClimbCommand(climber, baseController.b(), armsController.b())
    );

    // Manipulator default command (not spinning, unless angled down)
    manipulator.setDefaultCommand( // -0.1, 0.0
      new ManipulatorCommand(manipulator, 0.0, 0.0, () -> elevator.getManipulatorAngle() < 0.1)
    );

    // Elevator default command (stowed)
    /*elevator.setDefaultCommand(
      new StowCommand(elevator)
    );

    // Stow elevator when roll limit exceeded
    isRollLimitExceededTrigger
      //.onTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.STOW))
      .onTrue(new NotificationCommand("Robot roll limit exceeded", "Elevator has been STOWED", NotificationLevel.WARNING));

    // Intake (driver right trigger)
    baseController.rightTrigger()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.INTAKE).repeatedly())
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // Reverse Intake (arms right bumper)  
    armsController.rightBumper()
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // Coral Outtake (arms right trigger)
    armsController.rightTrigger()
      .whileTrue(new ManipulatorCommand(manipulator, 0.29, 0.1, armsController.a()));

    // L1 (arms A)
    armsController.a()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.L1).repeatedly());

    // L2 (arms X)
    armsController.x()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.L2, true));

    // L3 (arms Y)
    armsController.y()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.L3, true));

    // L4 (arms B)
    armsController.b()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.L4, true));

    armsController.leftTrigger()
      .whileTrue(new AlgaeCommand(elevator, manipulator, ElevatorPositions.ALGAE_HIGH));*/

    // Temp testing code
    baseController.a()
      .whileTrue(new InstantCommand(() -> climber.runClimber(baseController.getLeftY()), climber).repeatedly())
      .whileFalse(new InstantCommand(() -> climber.runClimber(0.0), climber).repeatedly());

    // Match timer start/stop
    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand());

    // Test mode
    isTestTrigger
      .whileTrue(new TestCommand(elevator, manipulator));
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