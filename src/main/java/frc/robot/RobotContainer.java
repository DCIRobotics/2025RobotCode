// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import constants used in the container
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.OIConstants;

//import all the robot subsystems or other commands referenced by container.
import frc.robot.commands.SwerveJoystickCmd;
import frc.robot.commands.SwerveAutoAimCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CameraSystem;

//auto files- import all
import frc.robot.commands.LeftDriveAndDump;
import frc.robot.commands.RightDriveAndDump;
import frc.robot.commands.BowToReef;
import edu.wpi.first.cameraserver.CameraServerShared;

//import java.util.List;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are creatd here
  // create the drive base, and all other systems
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Climber climber = new Climber();
  private final Arm arm = new Arm();
  private final Intake intake = new Intake();
  private final CameraSystem cameraSystem =  new CameraSystem();
  
  // construct joysticks
  public Joystick operatorStick = new Joystick(OIConstants.kOperatorControllerPort);
  public Joystick driveStick = new Joystick(OIConstants.kDriverControllerPort);

  // Sendable chooser is the list containing our auto commands as options
  // we pick the auto routine we want to run from this list on the dashboard
  SendableChooser<Command> autoPicker = new SendableChooser<>();

  //construct all auto routine command objects. List them all here
  private Command LeftDriveAndDump = new LeftDriveAndDump(swerveSubsystem, arm, intake);
  private Command RightDriveAndDump = new RightDriveAndDump(swerveSubsystem, arm, intake);
  private Command BowToReef = new BowToReef(swerveSubsystem, arm, intake);

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // function that maps buttons to the commands they will call.
    configureBindings();    
    
    // add all the auto options to the sendable chooser
    autoPicker.addOption("Left Drive and Dump", LeftDriveAndDump);
    autoPicker.addOption("Right Drive and Dump", RightDriveAndDump);
    autoPicker.addOption("Bow to reef", BowToReef);

    autoPicker.addOption("Do nothing", null);
    //set the default auto to be to do nothing, unless we choose something else
    autoPicker.setDefaultOption("Bow to reef", BowToReef);

    //put the autoPicker object on the dashboard
    SmartDashboard.putData(autoPicker);

    /*the default command for the swerve system is to be driven by the joystick. 
    See the SwerveJoystickCmd file for details on how this command operates.
    The command takes 4 parameters:
    the swerveSubsystem, the X axis robot movement, the y axis robot movement, and robot rotation
    */
    swerveSubsystem.setDefaultCommand
    (new SwerveJoystickCmd(
      swerveSubsystem, 
      () ->(double) Math.pow(driveStick.getRawAxis(OIConstants.driverXAxis),3), 
      () ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverYAxis),3), 
      () ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverRotAxis),3),
      // parameter is boolean for robot vs. field oriented drive.
      // default is field oriented- remove the ! to make default be robot oriented
      () -> !(driveStick.getRawButton(OIConstants.robotOrientedDriveButton)),
      //parameter for drive slow button
      () -> (Boolean) driveStick.getRawButton(OIConstants.driverSlowDownButton)));
    

    //the default command for the arm is to set its position to stowed
    arm.setDefaultCommand(new RunCommand(() -> arm.stopArm(), arm));

    //default intake command is left trigger eats, right trigger shoots.
    // power determined by how far you push down trigger axes    
    intake.setDefaultCommand(new RunCommand(() -> 
        intake.spinRollers(
          operatorStick.getRawAxis(OIConstants.LEFT_TRIGGER_AXIS_0to1),
          operatorStick.getRawAxis(OIConstants.RIGHT_TRIGGER_AXIS_0to1))
          , intake));  
    

    //climbr default is to stop
    climber.setDefaultCommand(new RunCommand(() ->
      climber.stop(), climber));    
      }

  /**
   * Use this method to define your trigger->command mappings, aka which buttons do which things.
   * Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // structure for referencing buttons on driver stick, which is a regular joystick
    // create button object bind it to a command.
    
    // command to reset gyro
    new JoystickButton(driveStick, OIConstants.driverGyroResetButton).whileTrue(new RunCommand(() ->swerveSubsystem.zeroHeading(), swerveSubsystem));
        
    // coral dumper buttons
   new JoystickButton(operatorStick, OIConstants.RIGHT_BUMPER).whileTrue(new RunCommand(() ->intake.spinLittleRoller(), intake));
    
    // climber
    new JoystickButton(operatorStick, OIConstants.BUTTON_START).whileTrue(new RunCommand(() ->climber.climberGoUp(), climber));
    new JoystickButton(operatorStick, OIConstants.BUTTON_BACK).whileTrue(new RunCommand(() ->climber.climberUnwind(), climber));

    // arm
    new JoystickButton(operatorStick, OIConstants.LEFT_BUMPER).onTrue(new RunCommand(() ->arm.resetArmEncoder(), arm));    
    new JoystickButton(operatorStick, OIConstants.BUTTON_Y).onTrue(new RunCommand(() ->arm.setArmPosition(ArmConstants.armStowedEncoderPosition), arm));
    new JoystickButton(operatorStick, OIConstants.BUTTON_B).whileTrue(new RunCommand(() ->arm.setArmPosition(ArmConstants.armEatFromReefPosition), arm));
    new JoystickButton(operatorStick, OIConstants.BUTTON_A).onTrue(new RunCommand(() ->arm.setArmPosition(ArmConstants.armEatFromFloorPosition), arm));
    new JoystickButton(operatorStick, OIConstants.BUTTON_X).onTrue(new RunCommand(() ->arm.setArmPosition(ArmConstants.armEatFromLollipop), arm));
    new POVButton(operatorStick, 180).whileTrue(new RunCommand(() ->arm.armManualUp(), arm));
    new POVButton(operatorStick, 0).whileTrue(new RunCommand(() ->arm.armManualDown(), arm));

    // command to auto seek processor april tag by camera using translation only
    new JoystickButton(driveStick, OIConstants.BUTTON_A)
        .whileTrue
        (
          new SwerveAutoAimCommand
          (
          swerveSubsystem, cameraSystem,
          //input for robot x movement aka 'forward/backward'
          () ->cameraSystem.calculateForwardSpeedByCamera(), 
          //() ->(double) Math.pow(driveStick.getRawAxis(OIConstants.driverXAxis),3), 

          //input for y movement aka lateral strafe
          () ->(double) cameraSystem.calculateStrafeSpeedByCamera(),
          //() ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverYAxis),3), 
 
          //input for rotation movement- test this first
          //() ->(double) cameraSystem.calculateRotationSpeedByCamera(),
          () ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverRotAxis),3),
          
          //field oriented is fsalse bc camera is on robot
          () -> (false)
          )
        );
      
        //command to auto rotate to april tag on processor
        new JoystickButton(driveStick, OIConstants.BUTTON_B)
        .whileTrue
        (
          new SwerveAutoAimCommand
          (
          swerveSubsystem, cameraSystem,
          //input for robot x movement aka 'forward/backward'
          () ->(double) Math.pow(driveStick.getRawAxis(OIConstants.driverXAxis),3), 

          //input for y movement aka lateral strafe
          () ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverYAxis),3), 
 
          //input for rotation movement- test this first
          () ->(double) cameraSystem.calculateRotationSpeedByCamera(),
          //() ->(double) -Math.pow(driveStick.getRawAxis(OIConstants.driverRotAxis),3),
          
          //field oriented is fsalse bc camera is on robot
          () -> (false)
          )
        );
                
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoPicker.getSelected();
  }
}


