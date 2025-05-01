// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// nothing can be commented here (joking! it's for the bit!)
package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RunCommand;
import java.util.List;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class ModuleConstants {
    //constants relate to physical module- encoders, gears etc.
    public static final double wheelDiameterMeters = 0.098;
    // this ratio might be 8.14:1- gotta check
    public static final double driveMotorGearRatio = 1/ 6.75;
    public static final double directionMotorGearRatio = 7.0/150;
    public static final double driveEncoderRotToMeter = driveMotorGearRatio * Math.PI * wheelDiameterMeters;
    public static final double driveEncoderRPMToMeterPerSecond = driveEncoderRotToMeter / 60;
    public static final double directionEncoderRotToRadian = directionMotorGearRatio * 2 * Math.PI;
    public static final double directionEncoderRPMToRadianPerSecond = directionEncoderRotToRadian / 60;

    //PID values for swerve motors- need to tune to make wheels rotate smoothly
    public static final double directionMotorP = 0.15;
    public static final double directionMotorI = 0;
    public static final double directionMotorD = 0;

    //PID values for swerve motors- need to tune to make wheels rotate smoothly
    public static final double driveMotorP = 0.1;
    public static final double driveMotorI = 0;
    public static final double driveMotorD = 0;
  }
  // constants related to driving the chassis, both in teleop and in auto
  public static class DriveConstants {
    //Distance between right and left wheels
    public static final double trackWidth = 0.57; //Meters
    //Distance between front and back wheels
    public static final double wheelBase = 0.57; //Meters
    
    // constants for chassis with bumpers
    public static final double robotLengthWithBumpers = 0.86;
    public static final double armExtendedLength = 0.38;
    public static final double optimalProcessorDistance = 0.71;
    //constructs an object that stores the location of the wheels on the robot chassis
    // referenced later to determine the speed of the robot moving across the floor.
    // left and forward are positive values, so that counter-clockwise rotation is
    // a positive number.
    public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(-wheelBase / 2, -trackWidth / 2), //front right module
            new Translation2d(wheelBase / 2, -trackWidth / 2), //rear right module
            new Translation2d(wheelBase / 2, trackWidth / 2), //rear left module
            new Translation2d(-wheelBase / 2, trackWidth / 2) //front left module
    );
    // sets max translational velocity, max angular velocit, and max auto
    // angular acceleration
    public static final double physicalMaxSpeedMeterPerSec = 4;
    public static final double physicalMaxAngularSpeedRadPerSec = 1 * Math.PI;
    public static final double autoMaxAngularAccelerationRadiansPerSecSquared = Math.PI/4; 

    
    //CAN IDs and boolean vars for SPARKMax module motors and CANCoders 
    
    //rear left module
    public static final int rearLeftDirectionMotorID = 1;
    public static final boolean rearLeftDirectionEncoderReversed = false;    
    public static final int rearLeftDriveMotorID = 2;
    public static final boolean rearLeftDriveEncoderReversed = false;
    public static final int rearLeftAbsEncoderID = 10;   
    public static final boolean rearLeftAbsEncoderReversed = true;

    // front left module
    public static final int frontLeftDirectionMotorID = 3;
    public static final boolean frontLeftDirectionEncoderReversed = false;
    public static final boolean frontLeftDriveEncoderReversed = false;   
    public static final int frontLeftDriveMotorID = 4;
    public static final int frontLeftAbsEncoderID = 11;
    public static final boolean frontLeftAbsEncoderReversed = true;    

    //front right module
    public static final int frontRightDirectionMotorID = 5;
    public static final boolean frontRightDirectionEncoderReversed = false;   
    public static final int frontRightDriveMotorID = 6;
    public static final boolean frontRightDriveEncoderReversed = false;
    public static final int frontRightAbsEncoderID = 12;
    public static final boolean frontRightAbsEncoderReversed = true;
   
    // rear right module
    public static final int rearRightDirectionMotorID = 7;
    public static final boolean rearRightDirectionEncoderReversed = false;    
    public static final int rearRightDriveMotorID = 8;
    public static final boolean rearRightDriveEncoderReversed = false;
    public static final int rearRightAbsEncoderID = 13;
    public static final boolean rearRightAbsEncoderReversed = true;


    /*  Offsets for each wheel, but we can fix this by 
    using the CTRE tuner*/
    public static final double frontRightAbsEncoderOffsetRad = 0;
    public static final double rearRightAbsEncoderOffsetRad = 0;
    public static final double rearLeftAbsEncoderOffsetRad = 0;
    public static final double frontLeftAbsEncoderOffsetRad = 0;

    //speed limits for teleop driving
    public static final double teleopSpeedLimit = 1;
    public static final double teleopRotationSpeedLimit =.85;

    //calculating max physics values for teleop using previous constants
    public static final double teleDriveMaxSpeedMetersPerSecond = physicalMaxSpeedMeterPerSec*teleopSpeedLimit;
    public static final double teleDriveMaxAngularSpeedRadiansPerSecond = physicalMaxAngularSpeedRadPerSec*teleopRotationSpeedLimit;
    public static final double teleDriveMaxAccelerationUnitsPerSecond = 2.5;
    public static final double teleDriveMaxAngularAccelerationUnitsPerSecond = 3;

    // constraints for the rotation of the robot in auto, the PID controller for theta variable
    public static final TrapezoidProfile.Constraints thetaControllerConstraints = 
      new TrapezoidProfile.Constraints
      (
        physicalMaxAngularSpeedRadPerSec,
        autoMaxAngularAccelerationRadiansPerSecSquared
      );    
  }

  // constants related to the operator/ joystick
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;

    // deadband means range of values that we want to ignore- to desensitize the joysticks
    // to tiny little movements.
    public static final double deadband = 0.02;

    //usb port for driver sticks. adding another stick means duplicate this variable
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;

    // names for all buttons and axes, so we don't need to remember their indices
    public static final int BUTTON_A = 1;
    public static final int BUTTON_B = 2;
    public static final int BUTTON_X = 3;
    public static final int BUTTON_Y = 4;
    public static final int LEFT_BUMPER = 5;
    public static final int RIGHT_BUMPER = 6;
    public static final int BUTTON_BACK = 7;
    public static final int BUTTON_START = 8;
    public static final int LEFT_STICK_BUTTON_CLICK = 9;
    public static final int RIGHT_STICK_BUTTON_CLICK = 10;
    public static final int LEFT_STICK_VERTICAL_AXIS = 1;
    public static final int LEFT_STICK_HORIZONTAL_AXIS = 0;
    public static final int LEFT_TRIGGER_AXIS_0to1 = 2;
    public static final int RIGHT_TRIGGER_AXIS_0to1 = 3;    
    public static final int RIGHT_STICK_HORIZONTAL_AXIS = 4;
    public static final int RIGHT_STICK_VERTICAL_AXIS = 5;

    // buttons and axes we're using in this program
    // we refer to them by their actual function on the robot itself
    public static final int driverYAxis = 0;
    public static final int driverXAxis = 1;
    public static final int driverRotAxis = RIGHT_STICK_HORIZONTAL_AXIS;
    public static final int robotOrientedDriveButton = RIGHT_BUMPER;
    public static final int driverGyroResetButton = BUTTON_START;
    public static final int driverSlowDownButton = LEFT_BUMPER;

  }

  
  public static class ClimberConstants{
      public static final int climberMotorPWM = 0;
      public static final int ratchetServoPWM = 1;
      
      public static final double ratchetServoLockingPosition = 0;
      public static final double ratchetServoOpenPosition = 0.5;

      public static final double climberUpwardPower = -0.75;
      public static final double climberUnWindPower = 0.1;
      public static final int climberEncoderDIOPort = 0;
      public static final double climberUnWoundPosition = 0.278;
      public static final double climberUpPosition = 0.822;

  }  

  public static class CameraSystemConstants{
    public static final int climberCameraUSB = 0;
    public static final double heightOfLimelightOnRobot = 0.401; //measured on 3/29
    public static final double absoluteHeightOfTarget = 1.216;

    public static final double desiredDistanceCameraToProcessor = 0.40; //this isn't real; there is roughly 30 cm of error
    //we don't have the time to figure out the error, so we just put in the distance that the dashboard reads for optimised scoring distance
    public static final double drivebyCamerakP = 0.2;
    public static final double rotatebyCamerakP = 0.005;
    public static final double drivebyCamerakPxdirection = 0.0075;
    // limelight upward tilt angle- measured in degrees and then converted to radians
    public static final double limelightMountingAngle = 36.5 * (Math.PI/180);
  } 

  public static class ArmConstants{
    public static final int armMotorCAN = 15;
    // limits for setpoints, based on arm encoder ticks
    public static final double armStowedEncoderPosition = 0;
    public static final double armVerticalPosition = 8.642;
    public static final double armEatFromLollipop = 18;
    public static final double armEatFromReefPosition = 12.99;
    public static final double armBowToReefPosition = 23.2;
    public static final double armEatFromFloorPosition = 28.06;    

    // limits for arm manual control movement, based on absolute encoder
    public static final double armLowestPossiblePosition = 0.057;
    public static final double armStowedPosition = 0.0312;
    public static final double armManualControlSlowDownFactor = 1.0/8;
    public static final int armEncoderDIOPort = 8;
    
    //PIDconstants for arm motion to setpoints
    public static double armP = 0.4;
    public static double armI = 0;
    public static double armD = 0.12;
    public static double armMaxVelocity = 50.0 * 60;
    public static double armMaxAccel = 3800.0;
    public static double armErrorTolerance = 0.5;

  }

  public static class IntakeConstants{
    public static final int bigRollerMotorCAN = 30;
    public static final int littleRollerCAN = 31;
    public static final double littleRollerEatPower = 0.75;
    public static final double littleRollerShootPower = -1;
    public static final double bigRollerEatPower = -0.4;
    public static final double bigRollerShootPower = 1;
    public static final double shootCoralPower = 0.3;
    public static final double holdAlgaePower = -0.2;
  }

  public static class CoralScorerConstants{
    public static final int level1ServoPWM = 4;
    public static final int level2ServoPWM = 5;

    public static final double level1ServoReceivePosition = 0.6;
    public static final double level1ServoScorePosition = 0.6;

    public static final double level2ServoReceivePosition = 0.55;
    public static final double level2TransportPosition = 0.75;
    public static final double level2ScorePosition = 0.3;

  }

  public class AutoConstants{
    public static final double initialWaitSeconds = 0;
        
  }
}
