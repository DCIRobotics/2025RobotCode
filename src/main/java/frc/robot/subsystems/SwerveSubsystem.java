package frc.robot.subsystems;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;


// imports for NAVX gyro
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

/* class to represent the chassis */
public class SwerveSubsystem extends SubsystemBase {

    
    /*attributes of a chassis. the chassis is all 4 modules and the gyro */
    private final SwerveModule frontRightModule = new SwerveModule(
            DriveConstants.frontRightDriveMotorID, 
            DriveConstants.frontRightDirectionMotorID, 
            DriveConstants.frontRightDriveEncoderReversed, 
            DriveConstants.frontRightDirectionEncoderReversed, 
            DriveConstants.frontRightAbsEncoderID, 
            DriveConstants.frontRightAbsEncoderOffsetRad, 
            DriveConstants.frontRightAbsEncoderReversed);

    private final SwerveModule rearRightModule = new SwerveModule(
            DriveConstants.rearRightDriveMotorID, 
            DriveConstants.rearRightDirectionMotorID, 
            DriveConstants.rearRightDriveEncoderReversed, 
            DriveConstants.rearRightDirectionEncoderReversed, 
            DriveConstants.rearRightAbsEncoderID, 
            DriveConstants.rearRightAbsEncoderOffsetRad, 
            DriveConstants.rearRightAbsEncoderReversed);

    private final SwerveModule rearLeftModule = new SwerveModule(
            DriveConstants.rearLeftDriveMotorID, 
            DriveConstants.rearLeftDirectionMotorID, 
            DriveConstants.rearLeftDriveEncoderReversed, 
            DriveConstants.rearLeftDirectionEncoderReversed, 
            DriveConstants.rearLeftAbsEncoderID, 
            DriveConstants.rearLeftAbsEncoderOffsetRad, 
            DriveConstants.rearLeftAbsEncoderReversed);

    private final SwerveModule frontLeftModule = new SwerveModule(
            DriveConstants.frontLeftDriveMotorID, 
            DriveConstants.frontLeftDirectionMotorID, 
            DriveConstants.frontLeftDriveEncoderReversed, 
            DriveConstants.frontLeftDirectionEncoderReversed, 
            DriveConstants.frontLeftAbsEncoderID, 
            DriveConstants.frontLeftAbsEncoderOffsetRad, 
            DriveConstants.frontLeftAbsEncoderReversed);
 
    // call to use green, older gyro
    // private ADXRS450_Gyro greenGyro = new ADXRS450_Gyro();
    
    // call to construct NavX gyro
    private AHRS purpleGyro = new AHRS(NavXComType.kMXP_SPI);
    
    private SwerveModulePosition [] modulePositions = new SwerveModulePosition[]{
        frontRightModule.getPosition(),
        rearRightModule.getPosition(),
        rearLeftModule.getPosition(),
        frontLeftModule.getPosition()
        };

    //This creates a new swerve odometer by taking the heading/location of the robot
    private final SwerveDriveOdometry odometer = new SwerveDriveOdometry(
        DriveConstants.driveKinematics, new Rotation2d(0), modulePositions);

    //constructor for a swerve drive chassis. no parameters needed because they
    //are all entered when the modules are constructed
    public SwerveSubsystem() {
        // separate thread for recalibrating gyro, wait 1 second before doing it
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
        }).start();
    }
    
    //method to zero the gyro- make sure to use name of gyro we are using
    public void zeroHeading() {
        purpleGyro.reset();
    }

    // get gyro heading- take remainder of dividing gyro angle by 360. 
    //gyro.getAngle() counts upwards continuously, so need to mod 360
    public double getHeading() {
        // if using green gyro, this needs to be gyro.getAngle()
        return Math.IEEEremainder(purpleGyro.getYaw(), 360);
    }

    /*turns the gyro heaing into a Rotation2D object
     * A rotation 2D object is an X-Y coordinate on the 
     * unit circle. For example, if heading is 30 degrees,
     * the Rotation2d will be 0.866, 0.5
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    // get the odometer as a "pose2D object" which is the X, Y, and angle of robot all as one object
    public Pose2d getPose2d(){
        return odometer.getPoseMeters();
    }

    
    // when called, resets the "pose" of the robot to be its current rot 2d, current moule positions,
    // and current pose- like resetting/ zero-ing out the gyro
    public void resetOdometry(Pose2d pose) {
        odometer.resetPosition(getRotation2d(), modulePositions, pose);
    }

    public double setSpeedLimit(Boolean robotIsFast){
        if(robotIsFast){
            return 1.0;
        }else{
            return 0.5;
        }
    }

    @Override
    public void periodic() {

        //constantly updates the odometer by getting the states of the 4 modules and the rotation 2d
        odometer.update(getRotation2d(), modulePositions);

        // put values related to chassis on dashboard
        SmartDashboard.putNumber("Robot Heading", getHeading());
        SmartDashboard.putNumber("Front Right Absolute Encoder", frontRightModule.getAbsEncoderRad());
        SmartDashboard.putNumber("Rear Right Absolute Encoder", rearRightModule.getAbsEncoderRad());
        SmartDashboard.putNumber("Rear Left Absolute Encoder", rearLeftModule.getAbsEncoderRad());
        SmartDashboard.putNumber("Front Left Absolute Encoder", frontLeftModule.getAbsEncoderRad());
        SmartDashboard.putString("rot2D", getRotation2d().toString());
        // current robot location on field
        SmartDashboard.putString("Robot Location", getPose2d().getTranslation().toString());
        SmartDashboard.putString("Pose2d", getPose2d().toString());
        
    }

    //helper method to stop all modules
    public void stopModules() {
        frontRightModule.stop();
        rearRightModule.stop();
        rearLeftModule.stop();
        frontLeftModule.stop();
    }

    /* set module states to  be their desired states,
    via the list of desired states 
    */  
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.physicalMaxSpeedMeterPerSec);
        frontRightModule.setDesiredState(desiredStates[0]);
        rearRightModule.setDesiredState(desiredStates[1]);
        rearLeftModule.setDesiredState(desiredStates[2]);
        frontLeftModule.setDesiredState(desiredStates[3]);
    }
}

