package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator implements Subsystem{
  private final SparkFlex manipulator = new SparkFlex(HardwareConstants.MANIPULATOR_CAN, MotorType.kBrushless);

  private final RelativeEncoder manipulatorEncoder = manipulator.getEncoder();
  private final SparkLimitSwitch algaeLimitSwitch = manipulator.getForwardLimitSwitch();
  //private final SparkAbsoluteEncoder manipulatorAbsoluteEncoder = manipulator.getAbsoluteEncoder();

  //Alert System
  private final Alert manipulatorAlert = new Alert("Manipulator Motor Error", AlertType.kError);

  // NT Output
  private final BooleanPublisher algaeLimitSwitchPub;

  // DataLog Output 
  private final DoubleLogEntry manipulatorVeloLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Velo", "RPM");
  private final DoubleLogEntry manipulatorPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Percentage", "%");
  private final BooleanLogEntry algaeLimitSwitchLog = 
    new BooleanLogEntry(DataLogManager.getLog(), "manipulator/algaeSwitch");

  public Manipulator() {
    manipulator.configure(ManipulatorConstants.MANIPULATOR_CONFIG,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("manipulator");

      algaeLimitSwitchPub = nt.getBooleanTopic("algaeLimitSwitch").publish();
    } else {
      algaeLimitSwitchPub = null;
    }
  }

  /**
   * @param velocity sets velo (in %) 
   */
  public void setManipulatorVelocity(double velocity) {
    manipulator.set(velocity);
    manipulatorPercentageLog.append(velocity);
  }

  /**
   * @return Whether the algae limit switch is pressed.
   */
  public boolean getAlgaeLimitSwitch() {
    return algaeLimitSwitch.isPressed();
  }
  
  public void log(){
    manipulatorVeloLog.append(manipulatorEncoder.getVelocity());
    algaeLimitSwitchLog.append(algaeLimitSwitch.isPressed());

    if (BuildConstants.PUBLISH_EVERYTHING) {
      algaeLimitSwitchPub.set(algaeLimitSwitch.isPressed());
    }
  }

  public void checkHardware(){
    manipulatorAlert.set(manipulator.hasActiveFault() || manipulator.hasActiveWarning());
  }
}
