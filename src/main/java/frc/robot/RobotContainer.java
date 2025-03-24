// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ElevatorPositions;
import frc.robot.Constants.LEDColors;
import frc.robot.commands.AlgaeCommand;
import frc.robot.commands.DeployClimberCommand;
import frc.robot.commands.ElevatorToStateCommand;
import frc.robot.commands.ManipulatorCommand;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.RetractClimberCommand;
import frc.robot.commands.StowCommand;
import frc.robot.commands.TeleopDriveCommand;
import frc.robot.commands.TestCommand;
import frc.robot.commands.alignment.LockOnCommand;
import frc.robot.commands.alignment.LockOnCommand.TargetGroup;
import frc.robot.commands.alignment.LockOnTargetChooserCommand;
import frc.robot.commands.alignment.LockOnTargetChooserCommand.TargetType;
import frc.robot.commands.led.SetLEDColorCommand;
import frc.robot.commands.led.SetLEDEnabledCommand;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.swerve.SwerveDrive;
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
  private final LED led = Util.timeInstantiation(LED::new);
  private final SwerveDrive base = Util.timeInstantiation(() -> new SwerveDrive(baseController::setRumble));
  private final Elevator elevator = Util.timeInstantiation(Elevator::new);
  private final Manipulator manipulator = Util.timeInstantiation(() -> new Manipulator(elevator::getManipulatorAngle));
  private final Climber climber = Util.timeInstantiation(Climber::new);

  // Dashboard
  private final AutoChooser auto = new AutoChooser(base, elevator, climber, manipulator, led);
  private final Dashboard dashboard = new Dashboard(baseController, armsController);
  private final Command pregameCommand = new PregameCommand(base, elevator, climber, auto);

  public RobotContainer() {
    // Add Pregame command to the dashboard
    SmartDashboard.putData("Pregame", pregameCommand);

    configureBindings();
  }

  private void configureBindings() {
    // Triggers
    final Trigger isEnabledTrigger = new Trigger(DriverStation::isEnabled);
    final Trigger isTestTrigger = new Trigger(DriverStation::isTestEnabled);
    final Trigger isRollLimitExceededTrigger = new Trigger(base::getRollLimitExceeded);

    // Inverted inputs
    final Supplier<Double> baseLeftX = () -> -baseController.getLeftX();
    final Supplier<Double> baseLeftY = () -> -baseController.getLeftY();
    final Supplier<Double> baseRightX = () -> -baseController.getRightX();

    // MARK: Default commands

    // SwerveDrive default command (teleop driving)
    base.setDefaultCommand(
      new TeleopDriveCommand(
        base, 
        baseLeftX, 
        baseLeftY, 
        baseRightX, 
        baseController.leftStick().or(baseController.leftTrigger()), 
        baseController.leftBumper())
    );
    
    // Manipulator default command (not spinning, unless angled down)
    manipulator.setDefaultCommand(
      new ManipulatorCommand(manipulator, -1.0, 0.0, () -> elevator.getManipulatorAngle() <= 0.1)
    );

    // Elevator default command (stowed)
    elevator.setDefaultCommand(
      new StowCommand(elevator)
    );

    // Climber default command (no moving)
    climber.setDefaultCommand(
      new InstantCommand(climber::stopClimber, climber).withName("ClimberDefaultCommand")
    );

    // LED default command (alliance color)
    led.setDefaultCommand(
      new SetLEDColorCommand(led, () -> Util.isRedAlliance(false) ? LEDColors.RED : LEDColors.BLUE)
    );

    // Stow elevator when roll limit exceeded
    isRollLimitExceededTrigger
      //.onTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.STOW))
      .onTrue(new NotificationCommand("Robot roll limit exceeded", "", NotificationLevel.WARNING))
      .whileTrue(led.flashWhiteFiveTimes.repeatedly());

    // Log file mark (drive Y)
    baseController.y()
      .onTrue(new PrintCommand("MARK"));

    // MARK: Driving alignment

    // Lock in to reef targets (base POV down, arms left stick down)
    baseController.povDown()
      .or(armsController.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.8))
      .onTrue(new LockOnTargetChooserCommand(TargetType.REEF));

    // All others are disabled
    // Lock in to coral station targets (base POV left, arms left stick left)
    /*baseController.povLeft()
      .or(armsController.axisLessThan(XboxController.Axis.kLeftX.value, -0.8))
      .onTrue(new LockOnTargetChooserCommand(TargetType.CORAL_STATION));

    // Lock in to cage targets (base POV up, arms left stick up)
    baseController.povUp()
      .or(armsController.axisLessThan(XboxController.Axis.kLeftY.value, -0.8))
      .onTrue(new LockOnTargetChooserCommand(TargetType.CAGE));

    // Lock in to algae targets (base POV right, arms left stick right)
    baseController.povRight()
      .or(armsController.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.8))
      .onTrue(new LockOnTargetChooserCommand(TargetType.ALGAE_SCORING));*/

    // Line up with left targets (base X)
    baseController.x()
      .whileTrue(new LockOnCommand(base, led, baseLeftX, baseLeftY, dashboard::setAllRumble, TargetGroup.LEFT));

    // Line up with center targets (base A)
    baseController.a()
    .whileTrue(new LockOnCommand(base, led, baseLeftX, baseLeftY, dashboard::setAllRumble, TargetGroup.CENTER));

    // Line up with right targets (base B)
    baseController.b()
      .whileTrue(new LockOnCommand(base, led, baseLeftX, baseLeftY, dashboard::setAllRumble, TargetGroup.RIGHT));

    // Intake (driver right trigger)
    baseController.rightTrigger()
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.INTAKE).repeatedly())
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // MARK: Arms scoring

    // Reverse Intake (arms right bumper)  
    armsController.rightBumper()
      .whileTrue(new ManipulatorCommand(manipulator, -0.1));

    // Coral Outtake (arms right trigger)
    armsController.rightTrigger()
      .whileTrue(new ManipulatorCommand(manipulator, 0.25, 0.1, armsController.a()));

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

    // High Algae Intake (arms POV up)
    armsController.povUp()
      .onTrue(new AlgaeCommand(elevator, manipulator, led, ElevatorPositions.ALGAE_HIGH));

    // Low algae Intake (arms POV down)
    armsController.povDown()
      .onTrue(new AlgaeCommand(elevator,manipulator, led, ElevatorPositions.ALGAE_LOW));

    // Processor algae position (arms POV left)
    armsController.povLeft().or(armsController.povUpLeft()).or(armsController.povDownLeft())
      .and(isEnabledTrigger) // So the driver can hold it down between auto and teleop period
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.PROCESSOR).repeatedly());

    // Barge algae position (arms POV right)
    armsController.povRight().or(armsController.povUpRight()).or(armsController.povDownRight())
      .whileTrue(new ElevatorToStateCommand(elevator, ElevatorPositions.BARGE).repeatedly());

    // Algae outtake (arms left trigger)
    armsController.leftTrigger()
      .whileTrue(new ManipulatorCommand(manipulator, -0.3));

    // MARK: Climbing

    // Climber deploy (arms right stick press)
    armsController.rightStick()
      .onTrue(new DeployClimberCommand(climber));

    // Climber deploy (arms left stick press)
    armsController.leftStick()
      .whileTrue(new RetractClimberCommand(climber, led))
      .onFalse(new InstantCommand(climber::stopClimber, climber));
    
    // MARK: Game mode triggers

    // Match timer start/stop
    isEnabledTrigger
      .onTrue(dashboard.getRestartTimerCommand())
      .onFalse(dashboard.getStopTimerCommand())
      .onTrue(new SetLEDEnabledCommand(led, true))
      .onFalse(new SetLEDEnabledCommand(led, false));

    // Test mode
    isTestTrigger
      .whileTrue(new TestCommand(elevator, manipulator, climber, () -> -armsController.getLeftY()));
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