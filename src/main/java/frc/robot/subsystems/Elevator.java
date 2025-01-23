package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;
public class Elevator implements Subsystem{
  public TalonFX elevatorMain = new TalonFX(HardwareConstants.ELEVATOR_MAIN_CAN, HardwareConstants.CANIVORE);
  public TalonFX elevatorFollower = new TalonFX(HardwareConstants.ELEVATOR_FOLLOWER_CAN, HardwareConstants.CANIVORE);
  public SparkFlex manipulatorPivot = new SparkFlex(HardwareConstants.MANIPULATOR_PIVOT_CAN, MotorType.kBrushless);

  public SparkClosedLoopController manipulatorPivotPIDController = manipulatorPivot.getClosedLoopController();
  public RelativeEncoder

  public 

  public Elevator(){
    elevatorMain.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    elevatorFollower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);

    elevatorFollower.setControl(new Follower(HardwareConstants.ELEVATOR_MAIN_CAN, false));


  }
}
