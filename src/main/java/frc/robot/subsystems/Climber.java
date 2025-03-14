package frc.robot.subsystems;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

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
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.BuildConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public final class Climber implements Subsystem {
  private final SparkFlex climberPivot = new SparkFlex(HardwareConstants.CLIMBER_PIVOT, MotorType.kBrushless);
  private final SparkFlex climberWinch = new SparkFlex(HardwareConstants.CLIMBER_WINCH, MotorType.kBrushless);

  private final RelativeEncoder climberPivotEncoder = climberPivot.getEncoder();
  private final RelativeEncoder climberWinchEncoder = climberWinch.getEncoder();

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

    // Init NT
    if (BuildConstants.PUBLISH_EVERYTHING) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("Climber");
      pivotPositionPub = nt.getDoubleTopic("PivotPosition").publish();
      winchPositionPub = nt.getDoubleTopic("WinchPosition").publish();
    } else {
      pivotPositionPub = null;
      winchPositionPub = null;
    }

    // Reset encoder
    homeClimberToBottom();

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
   * @return Pivot position (in rots, no gear ratio)
   */
  public double getPivotPosition() {
    return climberPivotEncoder.getPosition();
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

  public void homeClimberToBottom() {
    System.out.printf("Homed climber pivot from %f -> 0.1\n", climberPivotEncoder.getPosition());
    System.out.printf("Homed climber winch from %f -> 0.0\n", climberWinchEncoder.getPosition());

    climberPivotEncoder.setPosition(0.1);
    climberWinchEncoder.setPosition(0.0);
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
