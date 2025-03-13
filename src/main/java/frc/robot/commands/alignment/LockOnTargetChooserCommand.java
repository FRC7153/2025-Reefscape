package frc.robot.commands.alignment;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.InstantCommand;

public class LockOnTargetChooserCommand extends InstantCommand {
  public static enum TargetType { REEF, CAGE, CORAL_STATION, ALGAE_SCORING }

  // Shared target output
  private static final StringPublisher targetTypePub = 
    NetworkTableInstance.getDefault().getTable("Swerve/LockOn").getStringTopic("Type").publish();

  private static TargetType currentType;

  static {
    // Init target type
    setTargetType(TargetType.REEF);
  }

  /**
   * Sets the current type of target to use.
   * @param type The type of target.
   */
  public static void setTargetType(TargetType type) {
    currentType = type;
    targetTypePub.set(type.name());
    System.out.printf("Now locking on to targets of type %s\n", type.name());
  }

  /**
   * @return The current type of target.
   */
  public static TargetType getTargetType() {
    return currentType;
  }

  /**
   * Instant command to set the target type.
   * @param type The type to set to.
   */
  public LockOnTargetChooserCommand(TargetType type) {
    super(() -> setTargetType(type));
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }

  @Override
  public String getName() {
    return "LockOnTargetChooserCommand";
  }
}
