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

public class LeftDriveAndDump extends SequentialCommandGroup {
    //constructor for the ComplexAuto command
    public LeftDriveAndDump (SwerveSubsystem swerveSubsystem, Arm arm, Intake intake){
        
        // create trajectory config object, same for all trajectories, specifying max speed and accel
        TrajectoryConfig trajectoryConfig = new TrajectoryConfig
        (
            DriveConstants.teleDriveMaxSpeedMetersPerSecond/5,
            DriveConstants.teleDriveMaxAccelerationUnitsPerSecond
        ); 

        // Define PID controllers to correct trajectory errors for robot chassis
        PIDController xController = new PIDController(0.07, 0, 0);
        PIDController yController = new PIDController(0.07, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController
        (
            //P, I, D and theta constraints in that order
            2.5,
            0, 
            0, 
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
                    new Translation2d(-0.5,-0.01),
                    new Translation2d(-1.4,-0.02)
                ),
            //where to end
                new Pose2d(-1.9,0, Rotation2d.fromDegrees(60)),
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

        //repeat the two blocks above to make other trajectories to have muti-part
        //movement
        Trajectory prepForTeleop = TrajectoryGenerator.generateTrajectory
        (
            // where to start
                new Pose2d(0,0, Rotation2d.fromDegrees(60)),        
            // go through these intermediate points
                List.of(
                    //reverse away from processor
                    new Translation2d(1,0.1)                  
                ),
            //where to end- left across field.
                new Pose2d(1,0, Rotation2d.fromDegrees(180)),
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
            new WaitCommand(3),
            
            // tell robot it's initial pose 
            new InstantCommand(() ->swerveSubsystem.resetOdometry(goToReef.getInitialPose())),          
            
            //drive to reef and drop arm simultaneously
            new ParallelCommandGroup(
                goToReefCommand,
                new InstantCommand(() ->arm.setArmPosition(ArmConstants.armBowToReefPosition), arm)            
            ),

            // stop all swerve modules 
            new InstantCommand(()->swerveSubsystem.stopModules(), swerveSubsystem),
            
            // dump coral
            new RunCommand(()-> intake.scoreCoral(), intake).withTimeout(0.5),
            new InstantCommand(() -> intake.stopAll()),

            //re-rack arm to stow position
            new RunCommand(() ->arm.setArmPosition(ArmConstants.armStowedEncoderPosition), arm).withTimeout(3),
            
            prepForTeleopCommand

        );
    }            
}
