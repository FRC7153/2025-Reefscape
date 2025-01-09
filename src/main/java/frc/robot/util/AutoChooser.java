package frc.robot.util;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.auto.SimpleDriveAuto;
import frc.robot.commands.SwerveCharacterizationCommand;
import frc.robot.subsystems.swerve.SwerveDrive;

public final class AutoChooser {
    private static final Command noOpCommand = new PrintCommand("No-op auto selected.");

    public final SendableChooser<Supplier<Command>> chooser = new SendableChooser<>();
    private Command currentSelectedCommand = noOpCommand;

    public AutoChooser(SwerveDrive drive) {
        // On-change
        chooser.onChange((Supplier<Command> newAuto) -> {
            currentSelectedCommand = newAuto.get();
            System.out.printf("New auto loaded: %s\n", currentSelectedCommand.getName());
        });

        // Add default option
        chooser.setDefaultOption("No-op", () -> noOpCommand);

        // Add Drive SysId options
        chooser.addOption("SYSID Swerve: Q+", () -> new SwerveCharacterizationCommand(drive, true, true));
        chooser.addOption("SYSID Swerve: Q-", () -> new SwerveCharacterizationCommand(drive, true, false));
        chooser.addOption("SYSID Swerve: D+", () -> new SwerveCharacterizationCommand(drive, false, true));
        chooser.addOption("SYSID Swerve: D-", () -> new SwerveCharacterizationCommand(drive, false, false));

        // Drive characterization test auto
        chooser.addOption("Auto Drive Test", () -> new SimpleDriveAuto(drive));
    }

    public Command getCurrentSelectedCommand() {
        return currentSelectedCommand;
    }
}