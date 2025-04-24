// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//imports so the program can run- pulling from other files
package frc.robot.subsystems;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

//library imports- rev and spark
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;

// wpi imports- for swerve math and command structure
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//imports from swerve drive specialities

public class SwerveModule extends SubsystemBase {
  /* Instance variables of a swerve module. A swerve module object
   * physically contains two motors, the encoders for each of them,
   * and an absolute encoder. It also contains a PID controller, a
   * software object that we use to move the swerve module to its position
   * smoothly and efficiently.
   */
  
  private final SparkMax driveMotor;
  private final SparkMax directionMotor;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder directionEncoder;
  private final CANcoder absEncoder;

  private final SparkClosedLoopController directionPIDController;
  private final SparkClosedLoopController drivePIDController;

  private final Boolean absEncoderReversed;
  private final Double absEncoderOffsetRad;

  // variable to account for the gyro being mounted to the chassis in an offset/ skew fashion
  // try not to do this, bc it makes auto harder. ideally it should be 0
  private double chassisAngularOffset = 0;

  /*Method to construct a swerve module object, which takes in all the
   * necessary info to assign ports to all motors etc.
  */
  public SwerveModule(int driveMotorID, int directionMotorID, boolean driveMotorReversed, boolean directionMotorReversed, 
          int absEncoderID, double absEncoderOffset, boolean absEncoderReversed) {

            //create drive and direction motor objects
            driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
            directionMotor = new SparkMax(directionMotorID, MotorType.kBrushless);
                        
            // create encoders for both motors
            driveEncoder = driveMotor.getEncoder();
            directionEncoder = directionMotor.getEncoder();
            
            // create absolute encoder
            this.absEncoderOffsetRad = absEncoderOffset;
            this.absEncoderReversed = absEncoderReversed;
            absEncoder = new CANcoder(absEncoderID, "rio");            
            
            //construct PID controllers for both motors 
            directionPIDController = directionMotor.getClosedLoopController();
            drivePIDController = driveMotor.getClosedLoopController();            
            
            // create configurations for the motors
            SparkMaxConfig driveMotorConfig = new SparkMaxConfig();
            SparkMaxConfig directionMotorConfig = new SparkMaxConfig();
            
            // set info for drive motor config
            driveMotorConfig.inverted(driveMotorReversed);
            driveMotorConfig.idleMode(IdleMode.kBrake);
            driveMotorConfig.encoder.positionConversionFactor(ModuleConstants.driveEncoderRotToMeter);
            driveMotorConfig.encoder.velocityConversionFactor(ModuleConstants.driveEncoderRPMToMeterPerSecond);
            driveMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
            //drive motor gets P I D values, set in constants file
            driveMotorConfig.closedLoop.p(ModuleConstants.driveMotorP);
            driveMotorConfig.closedLoop.i(ModuleConstants.driveMotorI);
            driveMotorConfig.closedLoop.d(ModuleConstants.driveMotorD);                                  
            //set voltage so the drive motor never gets more than 12v 
            driveMotorConfig.voltageCompensation(12);
            // set current limit so it never gets more than 80 amps
            driveMotorConfig.smartCurrentLimit(80);
            
            //configure direction motor config
            directionMotorConfig.inverted(directionMotorReversed);
            directionMotorConfig.idleMode(IdleMode.kBrake);
            directionMotorConfig.encoder.positionConversionFactor(ModuleConstants.directionEncoderRotToRadian);
            directionMotorConfig.encoder.velocityConversionFactor(ModuleConstants.directionEncoderRPMToRadianPerSecond);
            directionMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);           
            //direction motor gets P I D values, set in constants file
            directionMotorConfig.closedLoop.p(ModuleConstants.directionMotorP);
            directionMotorConfig.closedLoop.i(ModuleConstants.directionMotorI);
            directionMotorConfig.closedLoop.d(ModuleConstants.directionMotorD);
            directionMotorConfig.voltageCompensation(12);
            directionMotorConfig.smartCurrentLimit(60);            
            
            // tells direction motor that -pi and pi rotations are the same place
            // might not need this?
            directionMotorConfig.closedLoop.positionWrappingEnabled(true);
            directionMotorConfig.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);

            // apply the configs to both motors
            driveMotor.configure(driveMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            directionMotor.configure(directionMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            
            resetEncoders();

          }

  // gets position of drive encoder, in unit of rotations of the motor  
  public double getDrivePosition() {
    return driveEncoder.getPosition();
  }

  //gets position of swerve motor, in unit of rotations of the motor
  public double getDirectionPosition() {
    return directionEncoder.getPosition();
  }

  //gets velocity of drive motor, in units of RPM of motor
  public double getDriveVelocity() {
    return driveEncoder.getVelocity();
  }
  
  //gets velocity of swerve motor, in units of RPM of motor
  public double getDirectionVelocity() {
    return directionEncoder.getVelocity();
  }

  //gets absolute encoder position in radians
  public double getAbsEncoderRad() {
    // gets absolute position in degrees, turns to radians, subtracts offset
    double angle = absEncoder.getAbsolutePosition().getValueAsDouble();
    angle *= 2* Math.PI;
    angle -= absEncoderOffsetRad;
    //returns the angle and flips sign if the absolute encoder is set to reversed mode
    return angle * (absEncoderReversed ? -1.0 : 1.0);

  }

  // resets both encoders
  public void resetEncoders() {
    // set drive encoder to have value 0
    driveEncoder.setPosition(0);
    // sets swerve motor encoder to be whatever value the absolute encoder currently has
    directionEncoder.setPosition(getAbsEncoderRad());
  }

  /*get the current state of a module. A SwerveModuleState is an object that holds
   * the velocity and the rotational orientation of the module at a given moment.
   * this call creates a new SM State object every time the get method is called, so it 
   * always tells you the state of the module right now.
  */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getDirectionPosition()-chassisAngularOffset));
  }

  // returns the current position of the module
  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      driveEncoder.getPosition(),
      new Rotation2d(directionEncoder.getPosition()- chassisAngularOffset)
    );
  }

  // sets the module to have a desired state
  public void setDesiredState(SwerveModuleState desiredState) {
    //if the desired state is to not be moving, just stop
    if (Math.abs(desiredState.speedMetersPerSecond) < 0.001) {
      stop();
      return;
    }
    //otherwise, optimize the state so the wheel rotates minimum amount
    desiredState.optimize(getState().angle);
    
    // then, give the drive motor the needed power to get the module to its desired speed
    drivePIDController.setReference(
      desiredState.speedMetersPerSecond*DriveConstants.physicalMaxSpeedMeterPerSec, ControlType.kVelocity);
    //give the direction motor the needed power to get to its desired position.
    directionPIDController.setReference(
      desiredState.angle.getRadians(), ControlType.kPosition);

    //print desired state AND actual state of the module on the dashboard
    SmartDashboard.putString("Swerve[" + absEncoder.getDeviceID() + "] desired state", desiredState.toString());
    SmartDashboard.putString("Swerve[" + absEncoder.getDeviceID() + "] actual state", getState().toString());

  }

  // stop method to set both motors to speed 0
  public void stop() {
    driveMotor.set(0);
    directionMotor.set(0);
  }
}
