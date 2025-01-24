package frc.robot.subsystems;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;
public class Elevator implements Subsystem{
  private final TalonFX elevatorMain = new TalonFX(HardwareConstants.ELEVATOR_MAIN_CAN, HardwareConstants.CANIVORE);
  private final TalonFX elevatorFollower = new TalonFX(HardwareConstants.ELEVATOR_FOLLOWER_CAN, HardwareConstants.CANIVORE);
  private final SparkFlex manipulatorPivot = new SparkFlex(HardwareConstants.MANIPULATOR_PIVOT_CAN, MotorType.kBrushless);

  public SparkClosedLoopController manipulatorPivotPIDController = manipulatorPivot.getClosedLoopController();
  public RelativeEncoder manipulatorPivotEncoder = manipulatorPivot.getEncoder();
  public AbsoluteEncoder manipulatorPivotAbsoluteEncoder = manipulatorPivot.getAbsoluteEncoder();

  private final PositionVoltage elevatorPositionRequest = new PositionVoltage(.0)
    .withOverrideBrakeDurNeutral(false)
    .withSlot(0);

  public Elevator(){
    elevatorMain.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    elevatorFollower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);

    elevatorFollower.setControl(new Follower(HardwareConstants.ELEVATOR_MAIN_CAN, false));

    manipulatorPivot.configure(ElevatorConstants.MANIPULATOR_PIVOT_CONFIG,
     ResetMode.kResetSafeParameters,
     PersistMode.kPersistParameters);
  }

  /**
   * @param position sets elevator to set position (in rots)
   */
  public void setElevatorPosition(double position){
    elevatorMain.setControl(elevatorPositionRequest.withPosition(position));
  }  

}
