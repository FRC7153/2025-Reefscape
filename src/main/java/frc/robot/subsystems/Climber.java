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
  private final SparkFlex climberLeader = new SparkFlex(HardwareConstants.CLIMBER_LEADER_CAN, MotorType.kBrushless);
  private final SparkFlex climberFollower = new SparkFlex(HardwareConstants.CLIMBER_FOLLOWER_CAN, MotorType.kBrushless);

  private final RelativeEncoder climberEncoder = climberLeader.getEncoder();

  //Alert Output
  private final Alert climberLeaderAlert = new Alert("Climber Leader Motor Alert", AlertType.kError);
  private final Alert climberFollowerAlert = new Alert("Climber Follower Motor Alert", AlertType.kError);

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
    climberLeader.configure(
      ClimberConstants.CLIMBER_LEADER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    climberFollower.configure(
      ClimberConstants.CLIMBER_FOLLOWER_CONFIG,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    // Reset encoder
    climberEncoder.setPosition(0.0);
  }

  /**
   * @param percentage Runs climber in percentage
   */
  public void runClimber(double percentage) {
    climberLeader.set(percentage);
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
    climberCurrentLog.append(climberLeader.getOutputCurrent());
  }

  public void checkHardware(){
    climberLeaderAlert.set(climberLeader.hasActiveFault() || climberLeader.hasActiveWarning());
    climberFollowerAlert.set(climberFollower.hasActiveFault() || climberFollower.hasActiveWarning());
  }
}
