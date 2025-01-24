package frc.robot.util.dashboard;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants.BuildConstants;
import frc.robot.autos.BackAndForthTestAuto;
import frc.robot.autos.SimpleDriveTestAuto;
import frc.robot.commands.SysIdCharacterizationCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwervePaths;

public final class AutoChooser {
  private static final Command noOpCommand = new PrintCommand("No-op auto selected.");

  private final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
  private Command currentLoadedCommand = null;

  private final Alert noAutoLoadedAlert = new Alert("No auto loaded yet (run pregame)", AlertType.kInfo);

  public AutoChooser(SwerveDrive drive) {
    // On change
    chooser.onChange((Supplier<Command> newAuto) -> {
      currentLoadedCommand = null;
      noAutoLoadedAlert.set(true);
    });

    // Add default option
    chooser.setDefaultOption("No-op", () -> noOpCommand); // TODO set starting pose here

    // Autos that are used for testing
    if (BuildConstants.INCLUDE_TEST_AUTOS) {
      // Add Drive SysId options
      chooser.addOption("SYSID Swerve Q+", () -> new SysIdCharacterizationCommand(drive.getModuleRoutine(), true, true));
      chooser.addOption("SYSID Swerve Q-", () -> new SysIdCharacterizationCommand(drive.getModuleRoutine(), true, false));
      chooser.addOption("SYSID Swerve D+", () -> new SysIdCharacterizationCommand(drive.getModuleRoutine(), false, true));
      chooser.addOption("SYSID Swerve D-", () -> new SysIdCharacterizationCommand(drive.getModuleRoutine(), false, false));

      // Drive characterization test auto
      chooser.addOption("Auto Drive Test", () -> new SimpleDriveTestAuto(drive));
      chooser.addOption("Back and Forth Drive Test", () -> new BackAndForthTestAuto(drive));
      chooser.addOption("Path 1 Test", 
        () -> SwervePaths.getFollowPathCommand(drive, "Path 1 Test", true).withName("Path 1 Test"));
      chooser.addOption("Rotate 1 Test", 
        () -> SwervePaths.getFollowPathCommand(drive, "Rotate 1 Test", true).withName("Rotate 1 Test"));
    }

    // Add to dashboard
    SmartDashboard.putData("Auto", chooser);
    noAutoLoadedAlert.set(true);
  }

  /**
   * Loads the currently selected command.
   */
  public void loadAutoCommand() {
    currentLoadedCommand = chooser.getSelected().get();
    System.out.printf("New auto loaded: %s\n", currentLoadedCommand.getName());
    noAutoLoadedAlert.set(false);
  }

  /**
   * @return The currently selected command, loading it if it's not already loaded.
   */
  public Command getCurrentSelectedCommand() {
    if (currentLoadedCommand == null) {
      loadAutoCommand();
    }

    return currentLoadedCommand;
  }
}
