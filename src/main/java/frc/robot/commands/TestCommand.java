package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class TestCommand extends Command {
  private final Elevator elevator;
  private final Manipulator manipulator;

  private DoubleEntry heightInput, angleInput, speedInput;

  /**
   * Command to run in test mode to find elevator presets
   * @param elevator
   * @param manipulator
   */
  public TestCommand(Elevator elevator, Manipulator manipulator) {
    this.elevator = elevator;
    this.manipulator = manipulator;

    addRequirements(elevator, manipulator);
  }

  @Override
  public void initialize() {
    if (heightInput == null) {
      // Create inputs
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("ElevatorTestMode");

      heightInput = nt.getDoubleTopic("height").getEntry(0.0);
      angleInput = nt.getDoubleTopic("angle").getEntry(0.2);
      speedInput = nt.getDoubleTopic("speed").getEntry(0.0);

      heightInput.set(0.0);
      angleInput.set(0.2);
      speedInput.set(0.0);
      System.out.println("Elevator test mode NT inputs instantiated.");
    }
  }

  @Override
  public void execute() {
    // If in test mode and enabled, set position
    if (DriverStation.isTestEnabled()) {
      elevator.setElevatorPosition(heightInput.get());
      elevator.setManipulatorPivotPosition(angleInput.get());
      manipulator.setManipulatorVelocity(speedInput.get());
    }
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
