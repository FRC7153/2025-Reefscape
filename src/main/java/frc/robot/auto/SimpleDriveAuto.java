package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class SimpleDriveAuto extends Command {
    private final SwerveDrive drive;

    public SimpleDriveAuto(SwerveDrive drive) {
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.drive(0.0, 1.5, 0.0, true, true);
    }

    @Override
    public void execute() {
        drive.drive(0.0, 1.5, 0.0, true, true);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("SimpleAuto Finished");
        drive.drive(0.0, 0.0, 0.0, false, false);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public String getName() {
        return "SimpleDriveAuto";
    }
}
