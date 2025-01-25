package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.ManipulatorConstants;

public class Manipulator implements Subsystem{
  private final SparkFlex manipulator = new SparkFlex(HardwareConstants.MANIPULATOR_CAN, MotorType.kBrushless);
  private final SparkClosedLoopController manipulatorPIDController = manipulator.getClosedLoopController();
  private final RelativeEncoder manipulatorEncoder = manipulator.getEncoder();

  public Manipulator(){
    manipulator.configure(ManipulatorConstants.MANIPULATOR_CONFIG,
    ResetMode.kResetSafeParameters,
    PersistMode.kPersistParameters);
  }

  /**
   * 
   * @param velocity sets velo 
   */
  public void setManipulatorVelocity(double velocity){
    manipulatorPIDController.setReference(velocity, ControlType.kVelocity);
  }


}
