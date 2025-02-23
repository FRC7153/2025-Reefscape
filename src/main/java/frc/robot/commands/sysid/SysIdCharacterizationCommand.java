package frc.robot.commands.sysid;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

public class SysIdCharacterizationCommand extends ConditionalCommand {
  private final boolean quasistatic, direction;

  /**
   * Runs Swerve Drive Characterization if the FMS is NOT connected.
   * @param drive
   * @param quasistatic true for quasistatic, false for dynamic
   * @param direction true for forward, false for backward
   */
  public SysIdCharacterizationCommand(SysIdRoutine routine, boolean quasistatic, boolean direction) {
    super(
      // FMS is connected, do not characterize:
      new PrintCommand("Will not characterize while FMS is connected!"),
      // FMS is not connected, run characterization (quasistatic or dynamic):
      quasistatic ?
        routine.quasistatic(direction ? Direction.kForward : Direction.kReverse)
        : routine.dynamic(direction ? Direction.kForward : Direction.kReverse),
      // Check if FMS is connected
      DriverStation::isFMSAttached
    );

    this.quasistatic = quasistatic;
    this.direction = direction;
  }

  @Override
  public String getName() {
    return String.format("SysID(quasistatic: %b, direction: %b)", quasistatic, direction);
  }
}
  
