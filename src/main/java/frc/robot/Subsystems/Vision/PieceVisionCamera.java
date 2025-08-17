package frc.robot.Subsystems.Vision;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.Elevator.ElevatorIOSparkMax;

public class PieceVisionCamera{
    private PhotonCamera camera;
    private PhotonTrackedTarget target;
    private double yawRotation;
    private double xTransform;
    private double IFOV = (Math.PI/2)/180;
    private double halfAlgae = 413/2; //413

    public PieceVisionCamera(String photonCameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(photonCameraName);
        yawRotation = robotToCamera.getRotation().getZ();
        xTransform = Math.abs(robotToCamera.getTranslation().getX());
    }

    public double getAlgaeDepthCameraRelative(PhotonTrackedTarget target) {
        List<TargetCorner> corners = target.getDetectedCorners();
        
        if (corners.size() < 2) {
            SmartDashboard.putNumber("algae depth", -1);
            return -1;
        }
    
        double minX = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE;
    
        for (TargetCorner corner : corners) {
            double xAngle = corner.x * IFOV;
            minX = Math.min(minX, xAngle);
            maxX = Math.max(maxX, xAngle);
        }
    
        double angularWidth = Math.toRadians(maxX - minX);
        double depth = halfAlgae / Math.tan(angularWidth / 2); 
    
        SmartDashboard.putNumber("algae depth", depth);
        return depth;
    }
    

    public double calculateRobotRelativeYaw(PhotonTrackedTarget target){
        //Positive is the far side of the camera and negative is the close side
        double oppositeSide =  getAlgaeDepthCameraRelative(target)*Math.cos(yawRotation - target.getPitch()); //was yaw
        double adjacentSide = getAlgaeDepthCameraRelative(target)*Math.sin(yawRotation - target.getPitch()) - xTransform; // was yaw
        return Math.atan2(oppositeSide,adjacentSide);
    }

    public double getCenterOffset(){
        PhotonPipelineResult result = camera.getLatestResult();
        
        if (result.hasTargets()){
            target = result.getBestTarget();
            return calculateRobotRelativeYaw(target);
        }
        return 0;
    }

    public boolean hasTargets(){
        return camera.getLatestResult().hasTargets();
    }
}
