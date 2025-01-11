package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class GoToPointCommand extends Command {
    PIDController pidXController =  new PIDController(0.1, 0, 0) ;
    PIDController pidYController = new PIDController(0.1, 0, 0);
    PIDController pidThetaController = new PIDController(0.1 , 0, 0);
   @Override
    public void initialize(){

    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }
}
