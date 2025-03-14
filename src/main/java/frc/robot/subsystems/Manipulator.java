package frc.robot.subsystems;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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
  private double velocityRequest;

  private final Supplier<Double> manipulatorAngleSupplier;

  //Alert System
  private final Alert manipulatorAlert = new Alert("Manipulator Motor Error", AlertType.kError);

  // NT Output
  private final DoublePublisher manipulatorCurrentPub;

  // DataLog Output 
  private final DoubleLogEntry manipulatorVeloLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Velo", "RPM");
  private final DoubleLogEntry manipulatorPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "manipulator/Percentage", "%");
  
  public Manipulator(Supplier<Double> manipulatorAngleSupplier) {
    this.manipulatorAngleSupplier = manipulatorAngleSupplier;

    manipulator.configure(ManipulatorConstants.MANIPULATOR_CONFIG,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("manipulator");

      manipulatorCurrentPub = nt.getDoubleTopic("manipulatorCurrent").publish();
    } else {
      manipulatorCurrentPub = null;
    }
  }

  /**
   * @param velocity sets velo (in %) 
   */
  public void setManipulatorVelocity(double velocity) {
    velocityRequest = velocity;
    //manipulator.set(velocity); // This is done in periodic()
    manipulatorPercentageLog.append(velocity);
  }

  @Override
  public void periodic() {
    // Safety disable manipulator
    if (manipulatorAngleSupplier.get() >= 0.35) {
      // Do not run manipulator
      manipulator.stopMotor();
    } else {
      // Run manipulator
      manipulator.set(velocityRequest);
    }
  }

  public void log(){
    manipulatorVeloLog.append(manipulatorEncoder.getVelocity());

    if (BuildConstants.PUBLISH_EVERYTHING) {
      manipulatorCurrentPub.set(manipulator.getOutputCurrent());
    }
  }

  public void checkHardware(){
    manipulatorAlert.set(manipulator.hasActiveFault() || manipulator.hasActiveWarning());
  }
}
