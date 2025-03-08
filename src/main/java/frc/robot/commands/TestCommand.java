package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class TestCommand extends Command {
  private final Elevator elevator;
  private final Manipulator manipulator;
  private final Climber climber;

  private DoubleEntry heightInput, angleInput, speedInput;
  private final Supplier<Double> climbWinchInput;

  /**
   * Command to run in test mode to find elevator presets
   * @param elevator
   * @param manipulator
   * @param climber
   * @param climbWinchInput -1 to 1.
   */
  public TestCommand(Elevator elevator, Manipulator manipulator, Climber climber, Supplier<Double> climbWinchInput) {
    this.elevator = elevator;
    this.manipulator = manipulator;
    this.climber = climber;

    this.climbWinchInput = climbWinchInput;

    addRequirements(elevator, manipulator, climber);
  }

  @Override
  public void initialize() {
    if (heightInput == null) {
      // Create inputs
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("ElevatorTestMode");

      heightInput = nt.getDoubleTopic("height").getEntry(0.0);
      angleInput = nt.getDoubleTopic("angle").getEntry(0.4);
      speedInput = nt.getDoubleTopic("speed").getEntry(0.0);

      heightInput.set(0.0);
      angleInput.set(0.4);
      speedInput.set(0.0);
      System.out.println("Elevator test mode NT inputs instantiated.");
    }

    // Set climber coast mode
    climber.setBrakeMode(false);
    climber.stopClimber();
  }

  @Override
  public void execute() {
    // Set position
    elevator.setElevatorPosition(heightInput.get());
    elevator.setManipulatorPivotPosition(angleInput.get());

    manipulator.setManipulatorVelocity(speedInput.get());

    climber.runClimberWinch(MathUtil.clamp(climbWinchInput.get() / 2.0, -1.0, 1.0));
  }

  @Override
  public void end(boolean interrupted) {
    // Stop everything
    manipulator.setManipulatorVelocity(0.0);
    climber.runClimberWinch(0.0);

    climber.setBrakeMode(true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
