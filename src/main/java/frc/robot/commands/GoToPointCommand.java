package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

public class GoToPointCommand extends Command {
    PIDController pidXController =  new PIDController(0.1, 0, 0) ;
    PIDController pidYController = new PIDController(0.1, 0, 0);
    PIDController pidThetaController = new PIDController(0.1 , 0, 0);

    private SwerveDrive drive;
    public Pose2d target;

    public GoToPointCommand (SwerveDrive drive, Pose2d target) {
        this.drive = drive;
        pidXController.setSetpoint(target.getX());

        addRequirements(drive);
    }
   @Override
    public void initialize(){
        
    }
    @Override
    public void execute(){
        Pose2d pose = drive.getAllianceRelativePose();
        drive.drive(
            pidYController.calculate(pose.getY()),
            pidXController.calculate(pose.getX()), 
            pidThetaController.calculate(pose.getRotation().getDegrees()), 
            true, 
            false);
    }
    @Override
    public void end(boolean interrupted){
        if (interrupted) {
            drive.drive(0, 0, 0, false, false);  
        }
    }
}
