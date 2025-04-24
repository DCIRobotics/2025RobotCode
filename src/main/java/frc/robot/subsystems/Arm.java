package frc.robot.subsystems;

// wpi imports for math, encoders etc.
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//rev imports for Spark Stuff
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;


//import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
    //instance variables- all hardware objects
    private final SparkFlex armMotor;
    private final RelativeEncoder armMotorEncoder;
    private final DutyCycleEncoder armThroughBoreEncoder; 
    private final SparkClosedLoopController armClosedLoopController;

    // constructor- creates specific case of our arm, with all
    // port numbers for everything
    public Arm(){
        armMotor = new SparkFlex(ArmConstants.armMotorCAN, MotorType.kBrushless);
        armThroughBoreEncoder= new DutyCycleEncoder(new DigitalInput(ArmConstants.armEncoderDIOPort));

        // configure arm motor
        SparkFlexConfig armMotorConfig = new SparkFlexConfig();
        //invert motor because spur gears reverse rotation
        armMotorConfig.inverted(true);
        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);           
        //set voltage so they never get more than 12v 
        armMotorConfig.voltageCompensation(12);
        // set current limit so they never get more than 60 amps
        armMotorConfig.smartCurrentLimit(60);        
        
        //configure arm motor SparkFlex encoder, closed loop controller
        armMotorEncoder = armMotor.getEncoder();
        armClosedLoopController = armMotor.getClosedLoopController();
        armMotorConfig.closedLoop.p(ArmConstants.armP);
        armMotorConfig.closedLoop.i(ArmConstants.armI);
        armMotorConfig.closedLoop.d(ArmConstants.armD);
        armMotorConfig.closedLoop.velocityFF(2/565);
        armMotorConfig.closedLoop.maxMotion.maxVelocity(ArmConstants.armMaxVelocity);
        armMotorConfig.closedLoop.maxMotion.maxAcceleration(ArmConstants.armMaxAccel);
        armMotorConfig.closedLoop.maxMotion.allowedClosedLoopError(ArmConstants.armErrorTolerance);


        //apply the configuration to arm Motor
        armMotor.configure(armMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    // sets arm position using max motion from Spark motor controller
    public void setArmPosition(double armDesiredPosition){
        armClosedLoopController.setReference(armDesiredPosition, SparkBase.ControlType.kMAXMotionPositionControl);
    }

    // control arm manually, using through bore encoder to establish boundaries
    //note- arm through bore encoder stowed position is 0.54, lowest position is
    // 0.05, so that's why < and > are seemingly reversed
    public void armManualUp(){
        armMotor.set(0.2);         
    }
    
    //arm manual down- same logic as manual up
    public void armManualDown(){
            armMotor.set(-0.2);    
    }    

    
    public void stopArm(){
        armMotor.set(0);
    }

    //method to reset encoder in case arm skips teeth from defensive hits
    public void resetArmEncoder(){
        armMotorEncoder.setPosition(0);
    }

    //get absolute arm position, from through bore
    public double getArmAbsolutePosition(){
        return armThroughBoreEncoder.get();
    }

    // get arm motor tick encoder position
    public double getArmEncoderPosition(){
        return armMotorEncoder.getPosition();
    }

    public double getArmEncoderVelocity(){
        return armMotorEncoder.getVelocity();
    }
    
    // publish all data about system to dashboard
    @Override
    public void periodic(){
        SmartDashboard.putNumber("arm encoder value", getArmEncoderPosition());
        SmartDashboard.putNumber("arm absolute position", getArmAbsolutePosition());
        SmartDashboard.putNumber("arm speed", getArmEncoderVelocity());

    }
}
