package frc.robot.subsystems;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public final class Climber implements Subsystem {
  private final SparkFlex climberPivot = new SparkFlex(HardwareConstants.CLIMBER_PIVOT, MotorType.kBrushless);
  private final SparkFlex climberWinch = new SparkFlex(HardwareConstants.CLIMBER_WINCH, MotorType.kBrushless);

  private final DutyCycleEncoder climberPivotAbsEncoder = new DutyCycleEncoder(HardwareConstants.CLIMBER_PIVOT_ENCODER_DUTY_CYCLE_DIO);
  private final RelativeEncoder climberWinchEncoder = climberWinch.getEncoder();

  // State
  private boolean hasDeployed = false;

  //Alert Output
  private final Alert climberPivotAlert = new Alert("Climber Pivot Motor Alert", AlertType.kError);
  private final Alert climberWinchAlert = new Alert("Climber Winch Motor Alert", AlertType.kError);

  //SysId Routine
  private SysIdRoutine climberPivotRoutine;

  // NT Output
  private final DoublePublisher winchPositionPub, pivotPositionPub;
 
  // DataLog Output
  private final DoubleLogEntry climberPivotPositionLog =
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/PivotPosition", "rots");
  private final DoubleLogEntry climberPivotCurrentLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/PivotCurrent", "amps");
  private final DoubleLogEntry climberPivotPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/PivotPercentage", "%");
    private final DoubleLogEntry climberWinchPositionLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/WinchPosition", "rots");
  private final DoubleLogEntry climberWinchCurrentLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/WinchCurrent", "amps");
  private final DoubleLogEntry climberWinchPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/WinchPercentage", "%");

  /**
   * Init
   */
  public Climber() {
    climberPivot.configure(
      ClimberConstants.CLIMBER_PIVOT_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    climberWinch.configure(
      ClimberConstants.CLIMBER_WINCH_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    climberPivotAbsEncoder.setInverted(ClimberConstants.CLIMBER_PIVOT_ABS_ENC_INVERTED);

    // Init NT
    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("Climber");
      pivotPositionPub = nt.getDoubleTopic("PivotPosition").publish();
      winchPositionPub = nt.getDoubleTopic("WinchPosition").publish();
    } else {
      pivotPositionPub = null;
      winchPositionPub = null;
    }

    setBrakeMode(true);
  }

  public SysIdRoutine getClimberPivotRoutine(){
    System.out.println("Starting URCL SysId Routine");
    DataLogManager.start();
    URCL.start();

    if (climberPivotRoutine == null) {
      climberPivotRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.0).per(Second), Volts.of(0.0), Seconds.of(25)), 
        new SysIdRoutine.Mechanism((voltage) -> {
          climberPivot.setVoltage(voltage.in(Volts));
        },
         null, this));
    }
    return climberPivotRoutine;
  }

  /**
   * @param percentage Runs climber in percentage
   */
  public void runClimberPivot(double percentage) {
    climberPivot.set(percentage);
    climberPivotPercentageLog.append(percentage);
  }

  public void runClimberWinch(double percentage){
    climberWinch.set(percentage);
    climberWinchPercentageLog.append(percentage);
  }

  public void setBrakeMode(boolean pivotBrake, boolean winchBrake) {
    climberPivot.configure(
      new SparkMaxConfig().idleMode(pivotBrake ? IdleMode.kBrake : IdleMode.kCoast), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters);
    
    climberWinch.configure(
      new SparkMaxConfig().idleMode(winchBrake ? IdleMode.kBrake : IdleMode.kCoast), 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kNoPersistParameters);

    System.out.printf("Climber brake mode set (pivot: %b, winch: %b)\n", pivotBrake, winchBrake);
  }

  public void setBrakeMode(boolean brake) {
    setBrakeMode(brake, brake);
  }
 
  /**
   * @return Pivot position (in rots, absolute on shaft)
   */
  public double getPivotPosition() {
    return MathUtil.inputModulus(
      climberPivotAbsEncoder.get() + ClimberConstants.CLIMBER_PIVOT_ABS_ENC_OFFSET, 
      0.0, 
      1.0);
  }

  /**
   * @return Winch position (in rots, no gear ratio)
   */
  public double getWinchPosition() {
    return climberWinchEncoder.getPosition();
  }

  public void stopClimber(){
    runClimberPivot(0.0);
    runClimberWinch(0.0);
  }

  /**
   * Sets the has deployed flag to true.
   */
  public void setClimberHasDeployedFlag() {
    hasDeployed = true;
    System.out.println("ClimberHasDeployed flag has been set to true.");
  }

  /**
   * @return True, if the has deployed flag has been set to true
   */
  public boolean getClimberHasDeployedFlag() {
    return hasDeployed;
  }
  
  /**
   * logs the position of the left climber
   */
  public void log(){
    climberPivotPositionLog.append(getPivotPosition());
    climberPivotCurrentLog.append(climberPivot.getOutputCurrent());
    climberWinchPositionLog.append(climberWinchEncoder.getPosition());
    climberWinchCurrentLog.append(climberWinch.getOutputCurrent());

    if (BuildConstants.PUBLISH_EVERYTHING) {
      pivotPositionPub.set(getPivotPosition());
      winchPositionPub.set(getWinchPosition());
    }
  }

  public void checkHardware(){
    climberPivotAlert.set(climberPivot.hasActiveFault() || climberPivot.hasActiveWarning());
    climberWinchAlert.set(climberWinch.hasActiveFault() || climberWinch.hasActiveWarning());
  }
}
