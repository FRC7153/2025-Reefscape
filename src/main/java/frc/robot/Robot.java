// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.PregameCommand;
import frc.robot.util.CANLogger;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  public Robot() {
    // Configure CTRE SignalLogger
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    SignalLogger.setPath("/U/CTRE_Signal_Logger");

    // Init logging
    DataLogManager.logNetworkTables(true);
    DataLogManager.logConsoleOutput(true);

    DriverStation.startDataLog(DataLogManager.getLog(), true);
    NetworkTableInstance.getDefault().startConnectionDataLog(DataLogManager.getLog(), "NTConnections");

    // Init CAN logging
    CANLogger canLogger = new CANLogger(HardwareConstants.RIO_CAN, HardwareConstants.CANIVORE);
    canLogger.start();

    // Init robot base
    m_robotContainer = new RobotContainer();

    // Add logging periodic
    addPeriodic(m_robotContainer::log, 0.1, 0.001); // every 100 ms
    addPeriodic(m_robotContainer::checkHardware, 0.5, 0.001); // every 500 ms
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // Check pregame
    if (!PregameCommand.getHasPregamed()) {
      DriverStation.reportError("No pregame before autonomousInit()!", false);
      m_robotContainer.getPregameCommand().schedule();
    }

    // Run auto command
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    // Cancel auto
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Check pregame
    if (!PregameCommand.getHasPregamed()) {
      DriverStation.reportError("No pregame before teleopInit()!", false);
      m_robotContainer.getPregameCommand().schedule();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}