package frc.robot.subsystems;

// wpi imports for math, encoders etc.

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

//rev imports for Spark Stuff
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {
    //instance variables- all hardware objects
    private final SparkMax littleRoller;
    private final SparkMax bigRoller;

    // constructor- creates specific case of our arm, with all
    // port numbers for everything
    public Intake(){
        littleRoller = new SparkMax(IntakeConstants.littleRollerCAN, MotorType.kBrushed);
        bigRoller = new SparkMax(IntakeConstants.bigRollerMotorCAN, MotorType.kBrushed);

    }

    public void spinRollers(double input1, double input2){
        if(input1 > 0.1 && input2 < 0.1){
            littleRoller.set(input1);
            bigRoller.set(-input1);
        }else if (input2 > 0.1 && input1 < 0.1){
            littleRoller.set(-input2);
            bigRoller.set(input2);
        } else{
            littleRoller.set(0);
            bigRoller.set(0);
        }
    }
    
    public void autoEatAlgae(){
        littleRoller.set(-1);
        bigRoller.set(1);
    }
    public void autoScoreAlgae(){
        littleRoller.set(1);
        bigRoller.set(-1);
    }
    
    public void spinLittleRoller(){
        littleRoller.set(IntakeConstants.shootCoralPower);
    }

    public void scoreCoral(){
        bigRoller.set(IntakeConstants.shootCoralPower);
    }    

    public void holdAlgae(){
        bigRoller.set(IntakeConstants.holdAlgaePower);
    }  

    public double getLittleRollerPower(){
        return littleRoller.get();
    }
    public double getBigRollerPower(){
        return bigRoller.get();
    }
    public void stopAll(){
        littleRoller.set(0);
        bigRoller.set(0);       
    }




    
    // publish all data about system to dashboard
    @Override
    public void periodic(){
        SmartDashboard.putNumber("littler roller power", littleRoller.get());
        SmartDashboard.putNumber("big roller power", bigRoller.get());
       
    }
}
