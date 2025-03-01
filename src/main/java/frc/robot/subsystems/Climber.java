package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.urcl.URCL;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public class Climber implements Subsystem {
  private final SparkFlex climber = new SparkFlex(HardwareConstants.CLIMBER_CAN, MotorType.kBrushless);
  private final SparkFlex climberFollower = new SparkFlex(HardwareConstants.CLIMBER_FOLLOWER_CAN, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder = climber.getEncoder();

  //Alert Output
  private final Alert climberAlert = new Alert("Climber Leader Motor Alert", AlertType.kError);

  //SysId Routine
  private SysIdRoutine climberRoutine;
 
  // DataLog Output
  private final DoubleLogEntry climberPositionLog =
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/Position", "rots");
  private final DoubleLogEntry climberCurrentLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/Current", "amps");
  private final DoubleLogEntry climberPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "Climber/Percentage", "%");

  /**
   * Init
   */
  public Climber() {
    climber.configure(
      ClimberConstants.CLIMBER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    climberFollower.configure(
      ClimberConstants.CLIMBER_FOLLOW_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    // Reset encoder
    climberEncoder.setPosition(0.0);
  }

  public SysIdRoutine getClimberRoutine(){
    System.out.println("Starting URCL SysId Routine");
    DataLogManager.start();
    URCL.start();

    if (climberRoutine == null) {
      climberRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(Volts.of(0.0).per(Second), Volts.of(0.0), Seconds.of(25)), 
        new SysIdRoutine.Mechanism((voltage) -> {
          climber.setVoltage(voltage.in(Volts));
        },
         null, this));
    }
    return climberRoutine;
  }

  /**
   * @param percentage Runs climber in percentage
   */
  public void runClimber(double percentage) {
    climber.set(percentage);
    climberPercentageLog.append(percentage);
  }

  /**
   * @return Position (in rots)
   */
  public double getPosition() {
    return climberEncoder.getPosition() / ClimberConstants.CLIMBER_RATIO;
  }

  public void stopClimber(){
    climber.set(0.0);
  }
  /**
   * logs the position of the left climber
   */
  public void log(){
    climberPositionLog.append(getPosition());
    climberCurrentLog.append(climber.getOutputCurrent());
  }

  public void checkHardware(){
    climberAlert.set(climber.hasActiveFault() || climber.hasActiveWarning());
  }
}
