package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog.State;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.util.dashboard.HardwareFaultTracker;

public final class Elevator implements Subsystem {
  public static record ElevatorState(double height, double angle) {}

  private final TalonFX elevatorMain = new TalonFX(HardwareConstants.ELEVATOR_LEADER_CAN, HardwareConstants.CANIVORE);
  private final TalonFX elevatorFollower = new TalonFX(HardwareConstants.ELEVATOR_FOLLOWER_CAN, HardwareConstants.CANIVORE);
  private final TalonFX manipulatorPivot = new TalonFX(HardwareConstants.MANIPULATOR_PIVOT_CAN, HardwareConstants.RIO_CAN);

  private final SparkMax manipulatorSensors = new SparkMax(HardwareConstants.MANIPULATOR_SENSORS_CAN, MotorType.kBrushed);
  private final AbsoluteEncoder manipulatorAbsEncoder = manipulatorSensors.getAbsoluteEncoder();
  private final SparkLimitSwitch algaeLimitSwitch = manipulatorSensors.getForwardLimitSwitch();

  private final MotionMagicVoltage elevatorPositionRequest = new MotionMagicVoltage(0.0)
    .withOverrideBrakeDurNeutral(true)
    .withSlot(0);
  
  private final MotionMagicVoltage manipulatorPivotPositionRequest = new MotionMagicVoltage(0)
    .withOverrideBrakeDurNeutral(true)
    .withSlot(0);

  private final StaticBrake staticBrakeRequest = new StaticBrake();

  private final StatusSignal<Angle> elevatorPosition = elevatorMain.getPosition();
  private final StatusSignal<Angle> elevatorFollowerPosition = elevatorFollower.getPosition();
  private final StatusSignal<AngularAcceleration> elevatorAcceleration = elevatorMain.getAcceleration();
  private final StatusSignal<Angle> manipulatorPosition = manipulatorPivot.getPosition();
  
  private final Alert elevatorMainAlert = new Alert("Elevator Main Motor Alert", AlertType.kError);
  private final Alert elevatorFollowerAlert = new Alert("Elevator Follower Motor Alert", AlertType.kError);
  private final Alert manipulatorAlert = new Alert("Manipulator Pivot Motor Alert", AlertType.kError);
  private final Alert manipulatorNotHomedAlert = new Alert("Manipulator pivot failed to home", AlertType.kError);
  private final Alert manipulatorSensorsAlert = new Alert("Manipulator Sensors Spark Max Alert", AlertType.kError);

  private SysIdRoutine elevatorRoutine, manipulatorPivotRoutine;
  private boolean hasManipulatorHomed = false;

  // NT Logging
  private final DoublePublisher elevatorPositionPub, elevatorFollowerPositionPub, elevatorAccelerationPub, 
    elevatorSetpointPub, manipulatorPositionPub, manipulatorSetpointPub;
  private final BooleanPublisher algaeLimitSwitchPub;

  //DataLog
  private final DoubleLogEntry elevatorPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Position", "rotations");
  private final DoubleLogEntry elevatorSetpointLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Elevator/Setpoint", "rotations");
  private final DoubleLogEntry manipulatorPivotPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Position", "rotations");
  private final DoubleLogEntry manipulatorPivotSetPointLog =
    new DoubleLogEntry(DataLogManager.getLog(), "Manipulator/Setpoint", "rotations");
  private final BooleanLogEntry algaeLimitSwitchLog = 
    new BooleanLogEntry(DataLogManager.getLog(), "manipulator/algaeSwitch");

  /**
   * @param manipulator A reference to the manipulator subsystem, for homing.
   */
  public Elevator() {    
    elevatorMain.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);
    elevatorFollower.getConfigurator().apply(ElevatorConstants.ELEVATOR_CONFIG);

    elevatorFollower.setControl(new Follower(HardwareConstants.ELEVATOR_LEADER_CAN, false));

    manipulatorPivot.getConfigurator().apply(ElevatorConstants.MANIPULATOR_PIVOT_CONFIG);

    manipulatorSensors.configure(
      ElevatorConstants.MANIPULATOR_SENSOR_CONFIG, 
      SparkBase.ResetMode.kResetSafeParameters, 
      PersistMode.kPersistParameters);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("Elevator");

      elevatorPositionPub = nt.getDoubleTopic("elevatorPosition").publish();
      elevatorFollowerPositionPub = nt.getDoubleTopic("elevatorFollowerPosition").publish();
      elevatorAccelerationPub = nt.getDoubleTopic("elevatorAcceleration").publish();
      elevatorSetpointPub = nt.getDoubleTopic("elevatorSetpoint").publish();
      manipulatorPositionPub = nt.getDoubleTopic("manipulatorPosition").publish();
      manipulatorSetpointPub = nt.getDoubleTopic("manipulatorSetpoint").publish();
      algaeLimitSwitchPub = nt.getBooleanTopic("algaeLimitSwitch").publish();
    } else {
      elevatorPositionPub = null;
      elevatorFollowerPositionPub = null;
      elevatorAccelerationPub = null;
      elevatorSetpointPub = null;
      manipulatorPositionPub = null;
      manipulatorSetpointPub = null;
      algaeLimitSwitchPub = null;
    }

    // Reset the elevator's encoders
    resetElevatorEncoder();

    // Disable sensors-only spark max
    manipulatorSensors.disable();
  }

  /**
   * @param rotations sets elevator to set rotations (in rots). 0.0 is bottom, 4.5 is top.
   */
  public void setElevatorPosition(double rotations){
    // Sanity check rotations
    rotations = MathUtil.clamp(rotations, 0.0, 4.5);

    elevatorMain.setControl(elevatorPositionRequest.withPosition(rotations));
    elevatorSetpointLog.append(rotations);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      elevatorSetpointPub.set(rotations);
    }
  }

  /**
   * Stops the elevator.
   */
  public void stopElevator() {
    elevatorMain.setControl(staticBrakeRequest);
    elevatorSetpointLog.append(0.0);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      elevatorSetpointPub.set(0.0);
    }
  }

  public void resetElevatorEncoder() {
    elevatorPosition.refresh();
    System.out.printf("Reset elevator position from %f to 0.0\n", elevatorPosition.getValueAsDouble());

    elevatorMain.setPosition(0.0);
    elevatorFollower.setPosition(0.0);
  }
  
  /**
   * @param rotations sets Manipulator to position (in rotations). -0.25 to 0.4.
   */
  public void setManipulatorPivotPosition(double rotations) {
    // Do not run if the Falcon's encoder has not homed
    if (!hasManipulatorHomed) return;

    // Sanity check rotations
    rotations = MathUtil.clamp(rotations, -0.25, 0.4);

    manipulatorPivot.setControl(manipulatorPivotPositionRequest.withPosition(rotations));
    manipulatorPivotSetPointLog.append(rotations);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      manipulatorSetpointPub.set(rotations);
    }
  }

  /**
   * Stops the manipulator.
   */
  public void stopManipulatorPivot() {
    manipulatorPivot.setControl(staticBrakeRequest);
    manipulatorPivotSetPointLog.append(-1.0);

    if (BuildConstants.PUBLISH_EVERYTHING) {
      manipulatorSetpointPub.set(-1.0);
    }
  }

  /**
   * @param state State to set both the elevator height and arm angle.
   */
  public void setState(ElevatorState state) {
    setManipulatorPivotPosition(state.angle);
    setElevatorPosition(state.height);
  }

  /**
   * Sets the elevator motors to a certain duty cycle.
   * @param dutyCycle %, positive is up.
   */
  public void setHeightDutyCycle(double dutyCycle) {
    elevatorMain.set(dutyCycle);
  }

  /**
   * @return Elevator height, in rotations.
   */
  public double getElevatorHeight() {
    elevatorPosition.refresh();
    return elevatorPosition.getValueAsDouble();
  }

  /**
   * @return The manipulator angle, in rotations.
   */
  public double getManipulatorAngle() {
    manipulatorPosition.refresh();
    return manipulatorPosition.getValueAsDouble();
  }

  /**
   * @return Whether the algae limit switch is pressed.
   */
  public boolean getAlgaeLimitSwitch() {
    return algaeLimitSwitch.isPressed();
  }

  /**
   * gets value from manipulator Absolute Encoder
   */
  public void home(){
    manipulatorPosition.refresh();
    double currentPos = manipulatorPosition.getValueAsDouble();

    // If below 0.2, the sensor is likely disconnected. Use the default
    // position instead.
    double absPos = manipulatorAbsEncoder.getPosition();

    StatusCode resp = manipulatorPivot.setPosition(
      absPos > 0.2 ? absPos : ElevatorConstants.MANIPULATOR_PIVOT_DEFAULT_POS
    );
    System.out.printf("Homed manipulator pivot from %f -> %f\n", currentPos, ElevatorConstants.MANIPULATOR_PIVOT_DEFAULT_POS);

    boolean success = resp.equals(StatusCode.OK) && absPos > 0.2;
    manipulatorNotHomedAlert.set(!success);
    hasManipulatorHomed = true;
  }

  public SysIdRoutine getElevatorRoutine() {
    System.out.println("Starting CTRE SignalLogger due to getRoutine call in SwerveDrive");
    SignalLogger.start();

    if(elevatorRoutine == null){
      elevatorRoutine = new SysIdRoutine(
        // Use 0.1  as ramp and 0.35 as step if the test is run horizontally
        // Use 0.3 as ramp and 1.25 as step if the test is run vertically
        new SysIdRoutine.Config(Volts.of(0.3).per(Second), Volts.of(1.25), Seconds.of(25), (State state) -> {
          //Logging State
          SignalLogger.writeString("Elevator-SysID-State", state.toString());
        }), new SysIdRoutine.Mechanism((Voltage v) -> {
          elevatorMain.setVoltage(v.in(Volts));
        }, null, this)
      );
    }

    return elevatorRoutine;
  }

  public SysIdRoutine getManipulatorPivotRoutine() {
    System.out.println("Starting CTRE Signal Logger due to getRoutine call in SwerveDrive");
    SignalLogger.start();

    if(manipulatorPivotRoutine == null){
      manipulatorPivotRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.6).per(Second), Volts.of(0.5), null, (State state) -> {
          //Logging State
          SignalLogger.writeString("ManipulatorPivot-SysID-State", state.toString());
        }), new SysIdRoutine.Mechanism((Voltage v) -> {
          manipulatorPivot.setVoltage(v.in(Volts));
        }, null, this)
      );
    }

    return manipulatorPivotRoutine;
  }

  public void log() {
    elevatorPosition.refresh();
    manipulatorPosition.refresh();

    elevatorPositionLog.append(elevatorPosition.getValueAsDouble());
    manipulatorPivotPositionLog.append(manipulatorPosition.getValueAsDouble());
    
    algaeLimitSwitchLog.append(getAlgaeLimitSwitch());

    if (BuildConstants.PUBLISH_EVERYTHING) {
      elevatorFollowerPosition.refresh();
      elevatorAcceleration.refresh();
      
      elevatorPositionPub.set(elevatorPosition.getValueAsDouble());
      elevatorAccelerationPub.set(elevatorAcceleration.getValueAsDouble());
      manipulatorPositionPub.set(manipulatorPosition.getValueAsDouble());
      elevatorFollowerPositionPub.set(elevatorFollowerPosition.getValueAsDouble());

      algaeLimitSwitchPub.set(getAlgaeLimitSwitch());
    }
  }

  public void checkHardware(){
    HardwareFaultTracker.checkFault(elevatorMainAlert, !elevatorMain.isAlive() || !elevatorMain.isConnected());
    HardwareFaultTracker.checkFault(elevatorFollowerAlert, !elevatorFollower.isAlive() || !elevatorFollower.isConnected());
    HardwareFaultTracker.checkFault(manipulatorAlert, !manipulatorPivot.isAlive() || !manipulatorPivot.isConnected());
    HardwareFaultTracker.checkFault(manipulatorSensorsAlert, manipulatorSensors.getFaults().can);
  }
}
