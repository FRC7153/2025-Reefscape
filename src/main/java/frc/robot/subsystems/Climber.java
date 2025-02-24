package frc.robot.subsystems;

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
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public class Climber implements Subsystem {
  private final SparkFlex climber = new SparkFlex(HardwareConstants.CLIMBER_CAN, MotorType.kBrushless);
  private final SparkFlex climb2 = new SparkFlex(HardwareConstants.CLIMBER_FOLLOWER_CAN, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder = climber.getEncoder();

  //Alert Output
  private final Alert climberAlert = new Alert("Climber Leader Motor Alert", AlertType.kError);

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

    climb2.configure(
      ClimberConstants.CLIMBER_FOLLOW_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters
    );

    // Reset encoder
    climberEncoder.setPosition(0.0);
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
