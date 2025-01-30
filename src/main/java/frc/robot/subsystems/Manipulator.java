package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkFlexExternalEncoder;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator implements Subsystem{
  private final SparkFlex manipulator = new SparkFlex(HardwareConstants.MANIPULATOR_CAN, MotorType.kBrushless);

  private final RelativeEncoder manipulatorEncoder = manipulator.getEncoder();
  private final SparkAbsoluteEncoder manipulatorAbsoluteEncoder = manipulator.getAbsoluteEncoder();

  //Alert System
  private final Alert manipulatorAlert = new Alert("Manipulator Motor Error", AlertType.kError);

  // DataLog Output 
  private DoubleLogEntry manipulatorVeloLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Velo", "RPM");
  private DoubleLogEntry manipulatorPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Percentage", "%");

  public Manipulator() {
    manipulator.configure(ManipulatorConstants.MANIPULATOR_CONFIG,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  /**
   * @param velocity sets velo (in %) 
   */
  public void setManipulatorVelocity(double velocity) {
    manipulator.set(velocity);
    manipulatorPercentageLog.append(velocity);
  }

  public void log(){
    manipulatorVeloLog.append(manipulatorEncoder.getVelocity());
  }

  public void checkHardware(){
    manipulatorAlert.set(manipulator.hasActiveFault() || manipulator.hasActiveWarning());
  }

}
