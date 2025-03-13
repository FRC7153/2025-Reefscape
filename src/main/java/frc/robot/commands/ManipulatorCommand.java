package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Manipulator;

public class ManipulatorCommand extends RepeatCommand {
  private final String name;

  /**
   * Runs the manipulator velocity (%) depending on the condition until canceled.
   * @param manipulator
   * @param ifTrue Speed if condition is true.
   * @param ifFalse Speed if condition is false.
   * @param condition Condition checked on initialization
   */
  public ManipulatorCommand(Manipulator manipulator, double ifTrue, double ifFalse, BooleanSupplier condition) {
    super(
      new InstantCommand(
        () -> manipulator.setManipulatorVelocity(condition.getAsBoolean() ? ifTrue : ifFalse),
        manipulator
      )
    );

    name = String.format("ManipulatorCommand(%.3f, %.3f)", ifTrue, ifFalse);
  }

  /**
   * Runs the manipulator velocity (%) until canceled.
   * @param manipulator
   * @param velocity
   */
  public ManipulatorCommand(Manipulator manipulator, double velocity) {
    super(
      new InstantCommand(() -> manipulator.setManipulatorVelocity(velocity), manipulator)
    );

    name = String.format("ManipulatorCommand(%.3f)", velocity);
  }

  @Override
  public String getName() {
    return name;
  }
}
