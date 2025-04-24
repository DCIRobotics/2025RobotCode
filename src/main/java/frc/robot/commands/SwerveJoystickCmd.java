// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SwerveSubsystem;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;

/** Command to drive swerve chassis with joystick */
public class SwerveJoystickCmd extends Command {
  //the command needs the following objects to work properly:
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final Supplier<Boolean> fieldOrientedFunction, robotIsSlow;
  //SlewRateLimiters put a limit on the rate of change of the joystick values, smoothing acceleration
  private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

  /**
   * Creates the command object itself
   *
   * @param subsystem The subsystem used by this command.
   * In this command, we pass four parameters. The subsystem itself, then three
   * double values we call x speed, y speed, turning speed. In robot container, these
   * three values are pulled from the joystick axes
   */
  public SwerveJoystickCmd(
    SwerveSubsystem swerveSubsystem, 
    Supplier<Double> xSpdFunction, 
    Supplier<Double> ySpdFunction, 
    Supplier<Double> turningSpdFunction,
    Supplier<Boolean> fieldOrientedFunction,
    Supplier<Boolean> robotIsSlow) {

    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;
    this.fieldOrientedFunction= fieldOrientedFunction;
    this.robotIsSlow = robotIsSlow;
    this.xLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAccelerationUnitsPerSecond);
    this.turningLimiter = new SlewRateLimiter(DriveConstants.teleDriveMaxAngularAccelerationUnitsPerSecond);
    
    /*addRequirements says that whatever subsytems you put in as parameters must be used
    by this command. Prevents two commands that act on the same subsystem from being
    called simultaneously */
    addRequirements(swerveSubsystem);
  }

  /*Called when the command is initially scheduled. Good for things like
   * checking sensors to ensure the command can be safely executed
  */
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //1. Get real-time joystick inputs
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningSpeed = turningSpdFunction.get();

    /*2. Ensure that if the Joystick doesn't center at 0, we will ignore small inputs 
    by using the deadband functionality.
    ?: syntax means if(question to ask) ? what to do if true : what to do if false.
    In this case it means is the xSpeed bigger than the deadband?
    if yes, set xSpeed. if not, set to 0
    */
    xSpeed = Math.abs(xSpeed) > OIConstants.deadband ? xSpeed : 0.0;
    ySpeed = Math.abs(ySpeed) > OIConstants.deadband ? ySpeed : 0.0;
    turningSpeed = Math.abs(turningSpeed) > OIConstants.deadband ? turningSpeed : 0.0;

    //3. Make the driving smoother by limiting acceleration w SlewRateLimiter 
    xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.teleDriveMaxSpeedMetersPerSecond* DriveConstants.teleopSpeedLimit;
    ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.teleDriveMaxSpeedMetersPerSecond* DriveConstants.teleopSpeedLimit;
    turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;

    //if driver wants to be in slowdown mode, cut all speeds by 0.3.
    // controlled by button assigned in robot container.
    if(robotIsSlow.get()){
      xSpeed *=0.3;
      ySpeed*=0.3;
      turningSpeed *=0.3;
    }
    /*4. Construct desired ChassisSpeeds object. This object takes your field-relative
     * speeds x, y, and rotation, and turns them into robot-relative values. This is why
     * it needs the rotation2D of the chassis, because this tells the robot which way it is facing
     * in 2D space.
     */
    ChassisSpeeds chassisSpeeds;
    if(fieldOrientedFunction.get()){
      //drive relative to field
      chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
    } else{ //drive relative to robot
      chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
    }  
    //5. Create a list of swerve moule states to hold chassis speeds objects for each module
    SwerveModuleState[] moduleStates = DriveConstants.driveKinematics.toSwerveModuleStates(chassisSpeeds);

    //6. Call the set module states method on the moduleStates list, which sets each module
    // to the desire state based on the chassis speed desired in step 5
    swerveSubsystem.setModuleStates(moduleStates);
    
  }

  // Called once the command ends or is interrupted- makes sure all modules stop
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  // Returns true when the command should end. This command should never end,
  // so isFinished always returns false.
  @Override
  public boolean isFinished() {
    return false;
  } 
}
