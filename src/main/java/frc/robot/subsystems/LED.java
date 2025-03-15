package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HardwareConstants;
import frc.robot.Constants.LEDColors;

public class LED extends SubsystemBase {
  private final Spark ledController = new Spark(HardwareConstants.LED_PWM_PORT);
  private final PowerDistribution pdh = new PowerDistribution(HardwareConstants.PDH_CAN, ModuleType.kRev);

  /**
   * Creates a new ledController.
   */
  public LED() {
    ledController.set(LEDColors.BLACK);
    pdh.setSwitchableChannel(false);
  }

  public void enableLED(boolean enabled){
    pdh.setSwitchableChannel(enabled);
  }

  public void setColor(double color) {
   ledController.set(color);
  }
}

