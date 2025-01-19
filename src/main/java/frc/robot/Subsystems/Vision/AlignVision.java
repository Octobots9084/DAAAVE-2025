package frc.robot.Subsystems.Vision;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class AlignVision extends SubsystemBase {

    private static AlignVision INSTANCE;
    
	public static AlignVision getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new AlignVision();
		}
		return INSTANCE;
	}
    
    private PhotonCamera cam;
    private CANrangeConfiguration configuration;
    private FovParamsConfigs paramsConfigs;
    private CANrange rightRange;
    private CANrange leftRange;
    private boolean hasTargets;

    public AlignVision() {
        cam = new PhotonCamera("CamOne");
        leftRange = new CANrange(10);
        rightRange = new CANrange(12);

        paramsConfigs = new FovParamsConfigs();
        paramsConfigs.withFOVRangeX(6.75);
        paramsConfigs.withFOVRangeY(6.75);
        paramsConfigs.withFOVCenterX(6.75);
        paramsConfigs.withFOVCenterY(6.75);

        configuration = new CANrangeConfiguration();
        configuration.withFovParams(paramsConfigs);
        rightRange.getConfigurator().apply(configuration);
        leftRange.getConfigurator().apply(configuration);

    }

    @Override
	public void periodic() {
		

	}

    public double[] getReferenceRobotPosition(PhotonCamera camera) {
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        // Transform Tag Coordinates to Camera Coordinates from photonvision.
        Matrix<N4, N4> transformTagToCamera;
        
		if (!results.isEmpty()) {
			var result = results.get(results.size() - 1);

            if (result.getBestTarget() != null && result.getBestTarget().getFiducialId() == 16 ) {
                // Position of the AprilTag in Robot Coordinates.
                Matrix<N4, N1> referenceRobotPosition = null;
                
                // Get transformation matrix from photonvision
                transformTagToCamera = result.getBestTarget().getBestCameraToTarget().toMatrix();

                // referenceTagPosition = new Matrix<>(Nat.N4(), Nat.N1(), new double[]{0.381, 0.1524, 0, 1});

                // Transform Tag Position into Robot Coordinates
                referenceRobotPosition = VisionConstants.transformFrontLeftToRobot.times(transformTagToCamera.times(VisionConstants.referenceTagPosition));
                return referenceRobotPosition.getData();

            } else {
                return new double[] {Double.NaN};
            }
            
		} else {
            return new double[] {Double.NaN};
        }

        
    }

    public PhotonCamera getCamera() {
        return cam;
    }

    public double getRightLidarDistance() {
        return rightRange.getDistance().getValueAsDouble();
    }

    public double getLeftLidarDistance() {
        return leftRange.getDistance().getValueAsDouble();
    }

    public boolean areBothLidarsValid() {
        return getRightLidarDetect() && getLeftLidarDetect();
    }

    public boolean getRightLidarDetect() {
        return rightRange.getIsDetected().getValue();
    }

    public boolean getLeftLidarDetect() {
        return leftRange.getIsDetected().getValue();
    }

    public boolean getHasTargets() {
        return hasTargets;
    }
}
