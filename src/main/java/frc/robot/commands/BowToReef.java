package frc.robot.commands;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
//other files on robot referenced by this command
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Intake;


//library calls to wpi stuff referenced by this command
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
import java.time.Instant;

// ArrayList bc use a list for the trajectory locations
import java.util.List;

//a Complex auto is a sequential command group, which is a series of commands
//that happen consecutively. It only contains a constructor method, as
//no other methods are necessary

public class BowToReef extends SequentialCommandGroup {
    //constructor for the ComplexAuto command
    public BowToReef (SwerveSubsystem swerveSubsystem, Arm arm, Intake intake){
        
        // create trajectory config object, specifying max speed and accel
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig
        (
            DriveConstants.teleDriveMaxSpeedMetersPerSecond/5,
            DriveConstants.teleDriveMaxAccelerationUnitsPerSecond
        ); 

        // second trajectory config for faster driving, if needed.
        TrajectoryConfig fastTrajectoryConfig = new TrajectoryConfig
        (
            DriveConstants.teleDriveMaxSpeedMetersPerSecond/2,
            DriveConstants.teleDriveMaxAccelerationUnitsPerSecond
        );        
        // Define PID controllers to correct trajectory errors for robot chassis- x, y, and theta for rotation
        PIDController xController = new PIDController(0.07, 0, 0.007);
        PIDController yController = new PIDController(0.07, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController
        (
            //P, I, D and theta constraints in that order
            0.9,
            0, 
            0.07, 
            DriveConstants.thetaControllerConstraints
        );
        thetaController.enableContinuousInput(-Math.PI, Math.PI);        
        
        // create trajectory object and send the location
        // points for the robot into the TrajectoryGenerator function
        //x positive is forwards
        // y positive is to the right
        Trajectory goToReef = TrajectoryGenerator.generateTrajectory
        (
            // where to start
                new Pose2d(0,0, Rotation2d.fromDegrees(0)),        
            // go through these intermediate points
                List.of(
                    new Translation2d(-0.6,-0.02),
                    new Translation2d(-0.8,0.-0.03)
                ),
            //where to end
                new Pose2d(-1.1,0.1, Rotation2d.fromDegrees(0)),
            // what trajectory config to use
                trajectoryConfig
        );
        
        // Create the swerve controller command that makes the bot
        // follow the trajectory we are going to give it
        SwerveControllerCommand goToReefCommand = 
        new SwerveControllerCommand
        (
            goToReef, 
            swerveSubsystem::getPose2d, 
            DriveConstants.driveKinematics, 
            xController, 
            yController, 
            thetaController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );

        Trajectory finishGoToReef = TrajectoryGenerator.generateTrajectory
        (
            // where to start
                new Pose2d(0,0, Rotation2d.fromDegrees(0)),        
            // go through these intermediate points
                List.of(
                    new Translation2d(-0.15,-0.02)
                ),
            //where to end
                new Pose2d(-0.43,0, Rotation2d.fromDegrees(0)),
            // what trajectory config to use
                trajectoryConfig
        ); 
 
        SwerveControllerCommand finishGoToReefCommand = 
        new SwerveControllerCommand
        (
            finishGoToReef, 
            swerveSubsystem::getPose2d, 
            DriveConstants.driveKinematics, 
            xController, 
            yController, 
            thetaController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );        

        Trajectory goToProcessor = TrajectoryGenerator.generateTrajectory
        (
            // where to start
                new Pose2d(0,0, Rotation2d.fromDegrees(0)),        
            // go through these intermediate points
                List.of(
                    //reverse away from reef, drive to processor
                    new Translation2d(-0.01,0),
                    new Translation2d(-0.08,1.5),
                    new Translation2d(-0.15, 2.5)                    
                ),
            //where to end- left across field, facing processor
                new Pose2d(-0.45,2.75, Rotation2d.fromDegrees(-90)),
            // what trajectory config to use
                trajectoryConfig
        );
        
        // Create the swerve controller command that makes the bot
        // follow the trajectory we are going to give it
        SwerveControllerCommand goToProcessorCommand = 
        new SwerveControllerCommand
        (
            goToProcessor, 
            swerveSubsystem::getPose2d, 
            DriveConstants.driveKinematics, 
            xController, 
            yController, 
            thetaController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        ); 
        
        //trajectory to move robot to a desirable location and facing a desirable direction for 
        // start of teleop

        Trajectory prepForTeleop = TrajectoryGenerator.generateTrajectory
        (
            // where to start
                new Pose2d(0,0, Rotation2d.fromDegrees(-90)),        
            // go through these intermediate points
                List.of(
                    //reverse away from processor
                    new Translation2d(0.5,0.1)                  
                ),
            //where to end- left across field.
                new Pose2d(1,0, Rotation2d.fromDegrees(-180)),
            // what trajectory config to use
                trajectoryConfig
        );
        

        SwerveControllerCommand prepForTeleopCommand = 
        new SwerveControllerCommand
        (
            prepForTeleop, 
            swerveSubsystem::getPose2d, 
            DriveConstants.driveKinematics, 
            xController, 
            yController, 
            thetaController, 
            swerveSubsystem::setModuleStates, 
            swerveSubsystem
        );        
            
                
        // add all the commands we need to our sequential command group, which
        // is what will actually happen when we call this auto program
        addCommands
        (
            // wait some time, TBD based on match
            new WaitCommand(AutoConstants.initialWaitSeconds),
            
            // zero gyro and set it to be based on the robot's initial location/ orientation
            new InstantCommand(() -> swerveSubsystem.zeroHeading()), 
            new InstantCommand(() ->swerveSubsystem.resetOdometry(goToReef.getInitialPose())),          
            
            //drive to reef and drop arm simultaneously
            new ParallelCommandGroup(
                goToReefCommand,
                new InstantCommand(() ->arm.setArmPosition(ArmConstants.armBowToReefPosition), arm)            
            ),

            // stop driving  
            new InstantCommand(()->swerveSubsystem.stopModules(), swerveSubsystem),
            
            //dump coral
            new RunCommand(()-> intake.scoreCoral(), intake).withTimeout(0.5),
            
            //bring arm to vertical height 
            new InstantCommand(() ->arm.setArmPosition(ArmConstants.armVerticalPosition), arm),            
            new WaitCommand(0.5),
            // drive forward the rest of the way to reef
            finishGoToReefCommand,
            
            //eat ball - turn on intake, move arm to reef, wait 1.5s for eating to occur
            new ParallelCommandGroup(
                new RunCommand(() ->intake.autoEatAlgae(), intake).withTimeout(2.5),            
                new RunCommand(() ->arm.setArmPosition(ArmConstants.armEatFromReefPosition), arm).withTimeout(2.5)            
            ),

            //re-rack arm to stow position while holding in the algae
            new ParallelCommandGroup(
                new RunCommand(() ->intake.holdAlgae(), intake).withTimeout(1.0),
                new RunCommand(() ->arm.setArmPosition(ArmConstants.armStowedEncoderPosition), arm).withTimeout(1.0)
            ),
            //stop rollers
            new InstantCommand(() -> intake.stopAll()),
            
                 
            // drive to processor, stop modules
            goToProcessorCommand,
            new InstantCommand(()->swerveSubsystem.stopModules(), swerveSubsystem),

            // drop arm, score algae
            new RunCommand(() ->arm.setArmPosition(ArmConstants.armEatFromFloorPosition), arm).withTimeout(1),
            new RunCommand(()-> intake.autoScoreAlgae(), intake).withTimeout(1),
            new InstantCommand(() -> intake.stopAll()),
            
            // re-rack arm
            new RunCommand(() ->arm.setArmPosition(ArmConstants.armStowedEncoderPosition), arm),

            // add command to have robot face a desirable direction    
            prepForTeleopCommand
            
        );
    }            
}
