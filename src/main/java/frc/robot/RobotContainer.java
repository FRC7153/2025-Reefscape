// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.LockOnCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TestCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.util.AlignmentVector;
import frc.robot.util.Util;
import frc.robot.util.dashboard.AutoChooser;
import frc.robot.util.dashboard.Dashboard;
import frc.robot.util.dashboard.NotificationCommand;
import libs.Elastic.Notification.NotificationLevel;

public final class RobotContainer {
  // Controllers
  private final CommandXboxController baseController = new CommandXboxController(0);
  private final CommandXboxController armsController = new CommandXboxController(1);

  // Subsystems
  private final SwerveDrive base = Util.timeInstantiation(() -> new SwerveDrive(baseController::setRumble));
  private final Manipulator manipulator = Util.timeInstantiation(Manipulator::new);
  private final Elevator elevator = Util.timeInstantiation(Elevator::new);
  private final Climber climber = Util.timeInstantiation(Climber::new);

  // Dashboard
  private final AutoChooser auto = new AutoChooser(base, elevator, climber);
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
    base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        () -> -baseController.getLeftX(), 
        () -> -baseController.getLeftY(), 
        () -> -baseController.getRightX(), 
        baseController.leftTrigger())
    );
    
    // Climber default command (climb if both buttons pressed)
    //climber.setDefaultCommand(
    ///  new ClimbCommand(climber, baseController.b(), armsController.b())
    //);

    // Manipulator default command (not spinning, unless angled down)
    manipulator.setDefaultCommand(
      new ManipulatorCommand(manipulator, -1.0, 0.0, () -> elevator.getManipulatorAngle() < 0.1)
    );

    // Elevator default command (stowed)
    elevator.setDefaultCommand(
      new StowCommand(elevator)
    );

    // Stow elevator when roll limit exceeded
    isRollLimitExceededTrigger
      //.onTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.STOW))
      .onTrue(new NotificationCommand("Robot roll limit exceeded", "Elevator has been STOWED", NotificationLevel.WARNING));

    // Line up with left targets (base X)
    baseController.x()
      .whileTrue(new LockOnCommand(base, baseController::getLeftY, dashboard::setAllRumble, AlignmentVector.LEFT_VECTORS));

    // Line up with center targets (base A)
    baseController.a()
      .whileTrue(new LockOnCommand(base, baseController::getLeftY, dashboard::setAllRumble, AlignmentVector.CENTER_VECTORS));

    // Line up with right targets (base B)
    baseController.b()
      .whileTrue(new LockOnCommand(base, baseController::getLeftY, dashboard::setAllRumble, AlignmentVector.RIGHT_VECTORS));

    // Intake (driver right trigger)
    baseController.rightTrigger()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.INTAKE).repeatedly())
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // Reverse Intake (arms right bumper)  
    armsController.rightBumper()
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // Coral Outtake (arms right trigger)
    armsController.rightTrigger()
      .whileTrue(new ManipulatorCommand(manipulator, 0.25, 0.1, armsController.a())); // .25 .1

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

    // High Algae Intake (arms left trigger)
    armsController.leftTrigger()
      .whileTrue(new AlgaeCommand(elevator, manipulator, ElevatorPositions.ALGAE_HIGH));

    // Temp testing code
    //baseController.a()
    //  .whileTrue(new InstantCommand(() -> climber.runClimberWinch(baseController.getLeftY()), climber).repeatedly())
    //  .whileFalse(new InstantCommand(() -> climber.runClimberWinch(0.0), climber).repeatedly())
    //  .whileTrue(new InstantCommand(() -> climber.runClimberPivot(-baseController.getRightY())).repeatedly())
    //  .whileFalse(new InstantCommand(() -> climber.runClimberPivot(0.0)).repeatedly());

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