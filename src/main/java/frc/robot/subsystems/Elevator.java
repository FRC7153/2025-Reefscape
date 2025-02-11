package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;

public class Elevator implements Subsystem {
  private final TalonFX elevatorMain = new TalonFX(HardwareConstants.ELEVATOR_MAIN_CAN, HardwareConstants.CANIVORE);
  private final TalonFX elevatorFollower = new TalonFX(HardwareConstants.ELEVATOR_FOLLOWER_CAN, HardwareConstants.CANIVORE);
  private final TalonFX manipulatorPivot = new TalonFX(HardwareConstants.MANIPULATOR_PIVOT_CAN, HardwareConstants.RIO_CAN);

  private final PositionVoltage elevatorPositionRequest = new PositionVoltage(.0)
    .withOverrideBrakeDurNeutral(false)
    .withSlot(0);
  
  private final PositionVoltage manipulatorPivotPositionRequest = new PositionVoltage(0)
    .withOverrideBrakeDurNeutral(true)
    .withSlot(0);

  private final StatusSignal<Angle> elevatorPosition = elevatorMain.getPosition();
  private final StatusSignal<Angle> manipulatorPosition = manipulatorPivot.getPosition();
  
  private final Alert elevatorMainAlert = new Alert("Elevator Main Motor Alert", AlertType.kError);
  private final Alert elevatorFollowerAlert = new Alert("Elevator Follower Motor Alert", AlertType.kError);
  private final Alert manipulatorAlert = new Alert("Manipulator Pivot Motor Alert", AlertType.kError);
  private final Alert manipulatorNotHomedAlert = new Alert("Manipulator pivot failed to home", AlertType.kError);

  private SysIdRoutine elevatorRoutine, manipulatorPivotRoutine;
  private final Manipulator manipulator;
  private boolean hasManipulatorHomed = false;

  //DataLog
  private final DoubleLogEntry elevatorPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Position", "rotations");
  private final DoubleLogEntry elevatorSetpointLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Setpoint", "rotations");
  private final DoubleLogEntry manipulatorPivotPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Position", "rotations");
  private final DoubleLogEntry manipulatorPivotSetPointLog =
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Setpoint", "rotations");

  /**
   * @param manipulator A reference to the manipulator subsystem, for homing.
   */
  public Elevator(Manipulator manipulator) {
    this.manipulator = manipulator;
    
    elevatorMain.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    elevatorFollower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);

    elevatorFollower.setControl(new Follower(HardwareConstants.ELEVATOR_MAIN_CAN, false));

    manipulatorPivot.getConfigurator().apply(ElevatorConstants.MANIPULATOR_PIVOT_CONFIG);
  }

  /**
   * @param rotations sets elevator to set rotations (in rots). 0.0 is bottom.
   */
  public void setElevatorPosition(double rotations){
    elevatorMain.setControl(elevatorPositionRequest.withPosition(rotations));
    elevatorSetpointLog.append(rotations);
  }  
  
  /**
   * @param rotations sets Manipulator to position (in rotations). 0.0 is horizontal.
   */
  public void setManipulatorPivotPosition(double rotations) {
    // Do not run if the Falcon's encoder has not homed
    if (!hasManipulatorHomed) return;

    manipulatorPivot.setControl(manipulatorPivotPositionRequest.withPosition(rotations));
    manipulatorPivotSetPointLog.append(rotations);
  }

  public SysIdRoutine getElevatorRoutine(Elevator elevator) {
    System.out.println("Starting CTRE SignalLogger due to getRoutine call in SwerveDrive");
    SignalLogger.start();

    if(elevatorRoutine == null){
      elevatorRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.25).per(Second), Volts.of(1.5), null, (State state) -> {
          //Logging State
          SignalLogger.writeString("Elevator-SysID-State", state.toString());
        }), new SysIdRoutine.Mechanism((Voltage v) -> {
          elevatorMain.setVoltage(v.in(Volts));
        }, null, this)
      );
    }

    return elevatorRoutine;
  }

  public SysIdRoutine getManipulatorPivotRoutine(Elevator elevator) {
    System.out.println("Starting CTRE Signal Logger due to getRoutine call in SwerveDrive");
    SignalLogger.start();

    if(manipulatorPivotRoutine == null){
      manipulatorPivotRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(null, null, null, (State state) -> {
          //Logging State
          SignalLogger.writeString("ManipulatorPivot-SysID-State", state.toString());
        }), new SysIdRoutine.Mechanism((Voltage v) -> {
          manipulatorPivot.setVoltage(v.in(Volts));
        }, null, this)
      );
    }

    return manipulatorPivotRoutine;
  }
  /**
   * gets value from manipulator Absolute Encoder
   */
  public void home(){
    manipulatorPosition.refresh();
    double currentPos = manipulatorPosition.getValueAsDouble();
    double newPos = manipulator.getManipulatorAbsolutePosition();

    StatusCode resp = manipulatorPivot.setPosition(newPos);
    System.out.printf("Homed manipulator pivot from %f -> %f\n", currentPos, newPos);

    manipulatorNotHomedAlert.set(!resp.equals(StatusCode.OK));
    hasManipulatorHomed = resp.equals(StatusCode.OK);
  }

  public void log() {
    elevatorPosition.refresh();
    manipulatorPosition.refresh();

    elevatorPositionLog.append(elevatorPosition.getValueAsDouble());
    manipulatorPivotPositionLog.append(manipulatorPosition.getValueAsDouble());
  }

  public void checkHardware(){
    elevatorMainAlert.set(!elevatorMain.isAlive() || !elevatorMain.isConnected());
    elevatorFollowerAlert.set(!elevatorFollower.isAlive() || !elevatorFollower.isConnected());
    manipulatorAlert.set(!manipulatorPivot.isAlive() || !manipulatorPivot.isConnected());
  }
}
