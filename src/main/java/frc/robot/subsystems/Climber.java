package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.*;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.HardwareConstants;

public class Climber {
    private SparkFlex climberLeft = new SparkFlex(HardwareConstants.CLIMBER_LEFT_CAN, MotorType.kBrushless);
    private SparkFlex climberRight = new SparkFlex(HardwareConstants.CLIMBER_RIGHT_CAN, MotorType.kBrushless);

    private RelativeEncoder climberLeftEncoder = climberLeft.getEncoder();
    private RelativeEncoder climberRightEncoder = climberRight.getEncoder();

    private SparkClosedLoopController climberLeftController, climberRightController;

    private SparkFlexConfig climberLeftConfig = new SparkFlexConfig();
    private SparkFlexConfig climberRightConfig = new SparkFlexConfig();

    /**
     * Init
     */
    public Climber(){
        climberLeftConfig.idleMode(IdleMode.kBrake);
        climberRightConfig.idleMode(IdleMode.kBrake);

        //TODO
        climberLeftConfig.inverted(false);
        climberRightConfig.inverted(false);

        climberLeftConfig.smartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);
        climberRightConfig.smartCurrentLimit(ClimberConstants.kCLIMBER_CURRENT_LIMIT);

        climberLeftController = climberLeft.getClosedLoopController();
        climberRightController = climberRight.getClosedLoopController();

        // Trouble adding pid slot 
        climberLeftConfig.closedLoop.pid(ClimberConstants.kCLIMBER_P,
        ClimberConstants.kCLIMBER_I,
        ClimberConstants.kCLIMBER_D);

        climberRightConfig.closedLoop.pid(ClimberConstants.kCLIMBER_P,
        ClimberConstants.kCLIMBER_I, 
        ClimberConstants.kCLIMBER_D);
    }    

    /**
     * Sets Climber Height(in rots)
     * @param leftHeight
     * @param rightHeight
     */
    public void setClimberHeight(double leftHeight, double rightHeight){
        climberLeftController.setReference(leftHeight, ControlType.kPosition);
        climberRightController.setReference(rightHeight, ControlType.kPosition);
    }


    };
