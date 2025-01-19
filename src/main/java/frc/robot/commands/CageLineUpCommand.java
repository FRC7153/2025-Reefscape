package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class CageLineUpCommand extends Command {
  private static final SendableChooser<Integer> cageChooser = new SendableChooser<>();

  static {
    cageChooser.setDefaultOption("LEFT", 0);
    cageChooser.addOption("CENTER", 1);
    cageChooser.addOption("RIGHT", 2);
  }

  /**
   * @return SendableChooser for the cage to put on the dashboard.
   */
  public static Sendable getCageChooser() {
    return cageChooser;
  }

  // PID Controllers
  private final PIDController pidYController = new PIDController(0, 0, 0);
  private final PIDController pidThetaController = new PIDController(0.0, 0.0, 0.0);

  private final SwerveDrive swerve;
  private final Supplier<Double> xSupplier;
  private int target = 0;

  /**
   * Lines up (y and theta only) with the selected Cage.
   * @param swerve
   */
  public CageLineUpCommand(SwerveDrive swerve, Supplier<Double> xSupplier) {
    this.swerve = swerve;
    this.xSupplier = xSupplier;

    pidThetaController.setSetpoint(180.0); // Rear facing the cage

    addRequirements(swerve);
  }

  @Override
  public void initialize() {
    target = cageChooser.getSelected();

    pidYController.reset();
    pidThetaController.reset();
  }

  @Override
  public void execute() {

  }
}
