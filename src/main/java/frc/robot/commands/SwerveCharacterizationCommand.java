package frc.robot.commands;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SwerveCharacterizationCommand extends ConditionalCommand {
    private final boolean quasistatic, direction;

    /**
     * Runs Swerve Drive Characterization if the FMS is NOT connected.
     * @param drive
     * @param quasistatic true for quasistatic, false for dynamic
     * @param direction true for forward, false for backward
     */
    public SwerveCharacterizationCommand(SwerveDrive drive, boolean quasistatic, boolean direction) {
        super(
            // FMS is connected, do not characterize:
            new PrintCommand("Will not characterize while FMS is connected!"),
            // FMS is not connected, run characterization:
            new SequentialCommandGroup(
                // Init CTRE SignalLogger:
                new InstantCommand(() -> {
                    SignalLogger.start();
                    System.out.println("Started CTRE SignalLogger for drive characterization");
                }),
                // Run characterization (quasistatic or dynamic):
                quasistatic ?
                    drive.getRoutine().quasistatic(direction ? Direction.kForward : Direction.kReverse)
                    : drive.getRoutine().dynamic(direction ? Direction.kForward : Direction.kReverse)
            ),
            // Check if FMS is connected
            DriverStation::isFMSAttached
        );

        this.quasistatic = quasistatic;
        this.direction = direction;
    }

    @Override
    public String getName() {
        return String.format("SwerveSysID(quasistatic: %b, direction: %b)", quasistatic, direction);
    }
}
    
