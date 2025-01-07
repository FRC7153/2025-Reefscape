package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TeleopDriveCommand extends Command {
    private final SwerveDrive drive;
    private final Supplier<Double> ySupplier, xSupplier, thetaSupplier;
    private final BooleanSupplier fastMode;

    /**
     * @param drive
     * @param ySupplier Left is +, percentage
     * @param xSupplier Forward is +, percentage
     * @param thetaSupplier CCW+, percentage
     * @param fastMode
     */
    public TeleopDriveCommand(
        SwerveDrive drive, 
        Supplier<Double> ySupplier, 
        Supplier<Double> xSupplier, 
        Supplier<Double> thetaSupplier,
        BooleanSupplier fastMode
    ) {
        this.drive = drive;
        this.ySupplier = ySupplier;
        this.xSupplier = xSupplier;
        this.thetaSupplier = thetaSupplier;
        this.fastMode = fastMode;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        double x = xSupplier.get();
        double y = ySupplier.get();
        double theta = thetaSupplier.get();

        // Apply deadbands
        x = Math.abs(x) > 0.075 ? x : 0.0;
        y = Math.abs(y) > 0.075 ? x : 0.0;
        theta = Math.abs(theta) > 0.075 ? x : 0.0;

        drive.drive(
            y * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
            x * (fastMode.getAsBoolean() ? SwerveConstants.FAST_TRANSLATIONAL_SPEED : SwerveConstants.SLOW_TRANSLATIONAL_SPEED), 
            theta * (fastMode.getAsBoolean() ? SwerveConstants.FAST_ROTATIONAL_SPEED : SwerveConstants.SLOW_ROTATIONAL_SPEED), 
            false, 
            true
        );
    }

    @Override
    public void end(boolean terminated) {
        drive.drive(0.0, 0.0, 0.0, false, false);
    }
}
