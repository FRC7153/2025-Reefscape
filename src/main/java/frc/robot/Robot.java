// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.util.WPILibVersion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.HardwareConstants;
import frc.robot.commands.PregameCommand;
import frc.robot.commands.util.NotRequiredWrapperCommand;
import frc.robot.util.Util;
import frc.robot.util.logging.CANLogger;
import frc.robot.util.logging.ConsoleLogger;

public final class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;
  private final CANLogger m_canLogger;

  public Robot() {
    ConsoleLogger.init();
    System.out.printf("Built with WPILib %s\n", WPILibVersion.Version);

    // Configure CTRE SignalLogger
    SignalLogger.enableAutoLogging(false);
    SignalLogger.stop();
    SignalLogger.setPath("/u/CTRE_Signal_Logger");

    // Init logging
    DataLogManager.logNetworkTables(true);
    DataLogManager.logConsoleOutput(true); // this is sometimes garbled

    DriverStation.startDataLog(DataLogManager.getLog(), true);
    NetworkTableInstance.getDefault().startConnectionDataLog(DataLogManager.getLog(), "NTConnections");

    // Init CAN logging
    m_canLogger = new CANLogger(HardwareConstants.RIO_CAN, HardwareConstants.CANIVORE);
    m_canLogger.start();

    // Init robot base
    m_robotContainer = Util.timeInstantiation(RobotContainer::new);

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
      ConsoleLogger.reportError("No pregame before autonomousInit()!");
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
  public void autonomousExit() {
    // Cancel auto
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
      m_autonomousCommand = null;
    }

    // Stop any commands Auto may have started
    NotRequiredWrapperCommand.stopAllWrapperCommands();
  }

  @Override
  public void teleopInit() {
    // Check pregame
    if (!PregameCommand.getHasPregamed()) {
      ConsoleLogger.reportError("No pregame before teleopInit()!");
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