package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Servo;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ClimberConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Climber extends SubsystemBase{
    private final Talon climberMotor;
    private final DutyCycleEncoder climberEncoder;
    private final Servo ratchetServo; 


    public Climber(){
        climberMotor = new Talon(ClimberConstants.climberMotorPWM);
        climberEncoder= new DutyCycleEncoder(new DigitalInput(ClimberConstants.climberEncoderDIOPort));
        ratchetServo = new Servo(ClimberConstants.ratchetServoPWM);
    }

    public double getClimberPower(){
        return climberMotor.get();
    }

    public void stop(){
        climberMotor.set(0);
        ratchetServo.set(ClimberConstants.ratchetServoLockingPosition);
    }

    public void climberGoUp(){
        climberMotor.set(ClimberConstants.climberUpwardPower);
        ratchetServo.set(ClimberConstants.ratchetServoLockingPosition);

    } 

    public void climberUnwind(){
        ratchetServo.set(ClimberConstants.ratchetServoOpenPosition);
        if(ratchetServo.get() == ClimberConstants.ratchetServoOpenPosition){
            climberMotor.set(ClimberConstants.climberUnWindPower);
        }

    }

    @Override
    public void periodic(){
        SmartDashboard.putNumber(" climber power", getClimberPower());
        SmartDashboard.putNumber(" climber encoder position", climberEncoder.get());

    } 

}