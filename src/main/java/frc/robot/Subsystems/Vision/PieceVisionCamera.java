package frc.robot.Subsystems.Vision;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Subsystems.Elevator.ElevatorIOSparkMax;

public class PieceVisionCamera{
    private PhotonCamera camera;
    private PhotonTrackedTarget target;
    private double yawRotation;
    private double xTransform;

    public PieceVisionCamera(String photonCameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(photonCameraName);
        yawRotation = Math.abs(robotToCamera.getRotation().getZ());
        xTransform = Math.abs(robotToCamera.getTranslation().getX());
    }

    public double getAlgaeDepthCameraRelative(PhotonTrackedTarget target){
        //TODO write this method based off of tested data.
        return 0.0;
    }

    public double calculateRobotRelativeYaw(PhotonTrackedTarget target){
        //Positive is the far side of the camera and negative is the close side
        double oppositeSide =  getAlgaeDepthCameraRelative(target)*Math.cos(yawRotation - target.getYaw());
        double adjacentSide = getAlgaeDepthCameraRelative(target)*Math.sin(yawRotation - target.getYaw()) - xTransform;
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
