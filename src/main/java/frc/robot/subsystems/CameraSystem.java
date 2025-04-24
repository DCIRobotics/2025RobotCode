package frc.robot.subsystems;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraSystemConstants;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.LimelightHelpers;

public class CameraSystem extends SubsystemBase {
    private final UsbCamera climberCamera = CameraServer.startAutomaticCapture(CameraSystemConstants.climberCameraUSB);
    private final VideoSink server = CameraServer.getServer();

    // values to construct limelight camera
    NetworkTableInstance netTable = NetworkTableInstance.getDefault(); 
    NetworkTable table = netTable.getTable("limelight");    

    public CameraSystem(){
        server.setSource(climberCamera);

    }

    public double calculateRangeToTarget(){
        LimelightHelpers.setPipelineIndex("", 0);
        double targetY = LimelightHelpers.getTY("");// vertical offset to target, degrees
        double angleofInclinationToTargetRadians = targetY * (Math.PI / 180); // convert angle to radians so can use Math.tan
        //correct for tilted camera
        angleofInclinationToTargetRadians += CameraSystemConstants.limelightMountingAngle;
        // triangle math: tangent of inclination angle = delta in height between camera and target / horizontal distance away
        double cameraDistanceToTarget = (CameraSystemConstants.absoluteHeightOfTarget - CameraSystemConstants.heightOfLimelightOnRobot) / Math.tan(angleofInclinationToTargetRadians); 
        return cameraDistanceToTarget;
    }

    // function to drive forward/ backward using camera ty data
    public double calculateForwardSpeedByCamera(){
        // if no valid target, send 0 speed, so nothing happens
        if(!LimelightHelpers.getTV("")){
            return 0.0;
        } else if(LimelightHelpers.getFiducialID("") == 3 || LimelightHelpers.getFiducialID("")==16){
        //otherwise calculate speed to go based on distance to target.
        //proprtionality constant for driving w camera
        double kp = CameraSystemConstants.drivebyCamerakP;
        // calculate error using limelight ty and desired distance
        // error positive means too far away from processsor, negative means too close
        double error = calculateRangeToTarget() - CameraSystemConstants.desiredDistanceCameraToProcessor;
        // drive speed is proportional to error and opposite sign
        double drivingSpeed = -(error* kp);
        //scale drive speed by max teleop driving speed
        drivingSpeed *= DriveConstants.teleDriveMaxSpeedMetersPerSecond;
        return drivingSpeed;
        } else{
            return 0.0;
        }
    }

    // function to rotate left/ right based on camera tx data
    public double calculateRotationSpeedByCamera(){
        if(!LimelightHelpers.getTV("")){
            return 0.0;
        } else if(LimelightHelpers.getFiducialID("") == 3 || LimelightHelpers.getFiducialID("")==16){
            double kp = CameraSystemConstants.rotatebyCamerakP;
            // rotation speed is proportional to error - check sign though
            double rotationSpeed = LimelightHelpers.getTX("")* kp;
            //scale rotation speed down by max teleop rotational speed
            rotationSpeed *= DriveConstants.teleDriveMaxAngularSpeedRadiansPerSecond;
            return rotationSpeed;
        }else{
            return 0.0;
        }
    }

    // function to strafe left/ right based on camera tx data
    public double calculateStrafeSpeedByCamera(){
        if(!LimelightHelpers.getTV("")){
            return 0.0;
        }else{
            //p constant for driving w camera
            double kp = CameraSystemConstants.drivebyCamerakPxdirection;
            // strafing speed directly prooprtional to tx. big x means target far away, must strafe faster. check sign
            double strafingSpeed = LimelightHelpers.getTX("")* kp;
            //scale speed to max teleop driving speed
            strafingSpeed *= DriveConstants.teleDriveMaxSpeedMetersPerSecond;
            return -strafingSpeed;
        }
    }    
    
    @Override
    public void periodic(){
        double targetX = LimelightHelpers.getTX(""); //horizontal offset to target, degrees
        double targetY = LimelightHelpers.getTY("");// vertical offset to target, degrees
        double targetA = LimelightHelpers.getTA(""); //area of target as % of image 
        boolean hasTarget = LimelightHelpers.getTV(""); // is there a valid target

        SmartDashboard.putNumber("target x", targetX);
        SmartDashboard.putNumber("target y", targetY);
        SmartDashboard.putNumber("target a", targetA);
        SmartDashboard.putBoolean("target valid", hasTarget);
        SmartDashboard.putNumber("distance to target", calculateRangeToTarget());

    }
    
}
