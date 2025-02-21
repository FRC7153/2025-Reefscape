package frc.robot.commands;

import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

public class ElevatorDefaultCommand extends Command {
  private final Elevator elevator;

  private DoubleEntry heightInput, angleInput;

  /**
   * The elevator's default/test command. Includes test functionality, only in
   * test mode.
   * @param elevator
   */
  public ElevatorDefaultCommand(Elevator elevator) {
    this.elevator = elevator;

    addRequirements(elevator);
  }

  @Override
  public void execute() {
    // If switched to test mode, create inputs
    if (heightInput == null && DriverStation.isTest()) {
      NetworkTable nt = NetworkTableInstance.getDefault().getTable("ElevatorTestMode");

      heightInput = nt.getDoubleTopic("height").getEntry(0.0);
      angleInput = nt.getDoubleTopic("angle").getEntry(0.0);

      heightInput.set(0.0);
      angleInput.set(0.0);
      System.out.println("Elevator test mode NT inputs instantiated.");
    }

    // If in test mode and enabled, set position
    if (DriverStation.isTestEnabled()) {
      elevator.setElevatorPosition(heightInput.get());
      elevator.setManipulatorPivotPosition(angleInput.get());
    }
  }
  
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
