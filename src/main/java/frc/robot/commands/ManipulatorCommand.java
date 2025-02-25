package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.Manipulator;

public class ManipulatorCommand extends WrapperCommand {
  /**
   * Runs the manipulator velocity (%) depending on the condition until canceled.
   * @param manipulator
   * @param ifTrue Speed if condition is true.
   * @param ifFalse Speed if condition is false.
   * @param condition Condition checked on initialization
   */
  public ManipulatorCommand(Manipulator manipulator, double ifTrue, double ifFalse, BooleanSupplier condition) {
    super(
      new ConditionalCommand(
        new InstantCommand(() -> manipulator.setManipulatorVelocity(ifTrue), manipulator).repeatedly(),
        new InstantCommand(() -> manipulator.setManipulatorVelocity(ifFalse), manipulator).repeatedly(),
        condition
      )
    );
  }

  /**
   * Runs the manipulator velocity (%) until canceled.
   * @param manipulator
   * @param velocity
   */
  public ManipulatorCommand(Manipulator manipulator, double velocity) {
    super(
      new InstantCommand(() -> manipulator.setManipulatorVelocity(velocity), manipulator).repeatedly()
    );
  }
}
