package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;

public class Elevator implements Subsystem {
  private final TalonFX elevatorMain = new TalonFX(HardwareConstants.ELEVATOR_MAIN_CAN, HardwareConstants.CANIVORE);
  private final TalonFX elevatorFollower = new TalonFX(HardwareConstants.ELEVATOR_FOLLOWER_CAN, HardwareConstants.CANIVORE);
  private final SparkFlex manipulatorPivot = new SparkFlex(HardwareConstants.MANIPULATOR_PIVOT_CAN, MotorType.kBrushless);

  public SparkClosedLoopController manipulatorPivotController = manipulatorPivot.getClosedLoopController();
  public RelativeEncoder manipulatorPivotEncoder = manipulatorPivot.getEncoder();
  public AbsoluteEncoder manipulatorPivotAbsoluteEncoder = manipulatorPivot.getAbsoluteEncoder();

  private final PositionVoltage elevatorPositionRequest = new PositionVoltage(.0)
    .withOverrideBrakeDurNeutral(false)
    .withSlot(0);

  private final StatusSignal<Angle> elevatorPosition = elevatorMain.getPosition();
  
  private final Alert elevatorAlert = new Alert("Elevator Alert", AlertType.kError);
  private final Alert manipulatorAlert = new Alert("Manipulator Alert", AlertType.kError);

  //DataLog
  private final DoubleLogEntry elevatorPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Position", "rotations");
  private final DoubleLogEntry elevatorSetpointLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Setpoint", "rotations");
  private final DoubleLogEntry manipulatorPivotPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Position", "rotations");
  private final DoubleLogEntry manipulatorPivotSetPointLog =
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Setpoint", "rotations");

  public Elevator() {
    elevatorMain.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    elevatorFollower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);

    elevatorFollower.setControl(new Follower(HardwareConstants.ELEVATOR_MAIN_CAN, false));

    manipulatorPivot.configure(ElevatorConstants.MANIPULATOR_PIVOT_CONFIG,
     ResetMode.kResetSafeParameters,
     PersistMode.kPersistParameters);
  }

  /**
   * @param rotations sets elevator to set rotations (in rots)
   */
  public void setElevatorPosition(double rotations){
    elevatorMain.setControl(elevatorPositionRequest.withPosition(rotations));
    elevatorSetpointLog.append(rotations);
  }  
  
  /**
   * @param rotations sets Manipulator to position (in rotations)
   */
  public void setManipulatorPivotPosition(double rotations) {
    manipulatorPivotController.setReference(rotations, ControlType.kPosition);
    manipulatorPivotSetPointLog.append(rotations);
  }

  public void log() {
    elevatorPositionLog.append(elevatorPosition.getValueAsDouble());
    manipulatorPivotPositionLog.append(manipulatorPivotAbsoluteEncoder.getPosition());
  }

  public void checkHardware(){
    elevatorAlert.set(!elevatorMain.isAlive() || !elevatorMain.isConnected());
    manipulatorAlert.set(manipulatorPivot.hasActiveFault() || manipulatorPivot.hasActiveWarning());
  }
}
