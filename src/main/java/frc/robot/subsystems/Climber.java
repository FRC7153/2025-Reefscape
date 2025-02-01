package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public class Climber implements Subsystem {
  private final SparkFlex climberLeft = new SparkFlex(HardwareConstants.CLIMBER_LEFT_CAN, MotorType.kBrushless);
  private final SparkFlex climberRight = new SparkFlex(HardwareConstants.CLIMBER_RIGHT_CAN, MotorType.kBrushless);

  private final RelativeEncoder climberLeftEncoder = climberLeft.getEncoder();

  //Alert Output
  private final Alert climberLeftAlert = new Alert("Climber Left Motor Alert", AlertType.kError);
  private final Alert climberRightAlert = new Alert("Climber Right Motor Alert", AlertType.kError);

  // DataLog Output
  private final DoubleLogEntry climberPositionLog =
    new DoubleLogEntry(DataLogManager.getLog(), "ClimberLeft/Position", "rots");
  private final DoubleLogEntry climberPercentageLog = 
    new DoubleLogEntry(DataLogManager.getLog(), "ClimberLeft/Percentage", "%");

  /**
   * Init
   */
  public Climber() {
    climberLeft.configure(
      ClimberConstants.CLIMBER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    climberRight.configure(
      new SparkFlexConfig()
        .follow(climberLeft, false),
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);
  }

  /**
   * @param percentage Runs climber in percentage
   */
  public void runClimber(double percentage) {
    climberLeft.set(percentage);
    climberPercentageLog.append(percentage);
  }

  /**
   * @return Position (in rots)
   */
  public double getPosition() {
    return climberLeftEncoder.getPosition() / ClimberConstants.CLIMBER_RATIO;
  }

  /**
   * logs the position of the left climber
   */
  public void log(){
    climberPositionLog.append(getPosition());
  }

  public void checkHardware(){
    climberLeftAlert.set(climberLeft.hasActiveFault() || climberLeft.hasActiveWarning());
    climberRightAlert.set(climberRight.hasActiveFault() || climberRight.hasActiveWarning());
  }
}
