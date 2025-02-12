package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ClimbCommand extends Command {
  private final Climber climber;
  private final BooleanSupplier aButton, bButton;

  private final Alert awaitingAlert = new Alert("Climb awaiting both driver's input...", AlertType.kInfo);

  /**
   * Runs the climber only when both buttons are pressed simultaneously. 
   * 
   * @param climber
   * @param aButton Button on first controller
   * @param bButton Button on second controller
   */
  public ClimbCommand(Climber climber, BooleanSupplier aButton, BooleanSupplier bButton) {
    this.climber = climber;
    this.aButton = aButton;
    this.bButton = bButton;

    addRequirements(climber);
  }

  @Override
  public void execute() {
    boolean a = aButton.getAsBoolean();
    boolean b = bButton.getAsBoolean();

    if (a && b) {
      // Both buttons are pressed
      awaitingAlert.set(false);
      climber.runClimber((climber.getPosition() <= 5.0) ? 0.2 : 0.0); // TODO add limit, speed
    } else if (a || b) {
      // Only one button is pressed
      awaitingAlert.set(true);
      climber.runClimber(0.0);
    } else {
      // No buttons pressed
      awaitingAlert.set(false);
      climber.runClimber(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    // Never finish
    return false;
  }

  @Override
  public String getName() {
    return "ClimbCommand";
  }
}
