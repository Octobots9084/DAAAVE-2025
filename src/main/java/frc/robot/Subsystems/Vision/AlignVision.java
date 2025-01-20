package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

public class AlignVision extends SubsystemBase {

  private static AlignVision INSTANCE;

  public static AlignVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AlignVision();
    }
    return INSTANCE;
  }

  private Swerve swerve;

  private PhotonCamera cam;
  private CANrangeConfiguration configuration;
  private FovParamsConfigs paramsConfigs;
  private CANrange rightRange;
  private CANrange leftRange;

  private PIDController pidController;
  private PIDController lidarPIDController;
  private PIDController cameraDepthPIDController;
  private PIDController gyroPIDController;

  private double aveLidarDist;
  private double diffLidarDist;
  private double[] refPosition;

  public AlignVision() {
    this.swerve = Swerve.getInstance();

    this.cam = new PhotonCamera("CamOne");
    this.leftRange = new CANrange(10);
    this.rightRange = new CANrange(12);

    this.pidController = new PIDController(2, 0, 0);
    this.lidarPIDController = new PIDController(2, 0, 0);
    this.cameraDepthPIDController = new PIDController(1.25, 0, 0);
    this.gyroPIDController = new PIDController(4, 0, 0);
    gyroPIDController.enableContinuousInput(0, 2 * Math.PI);

    this.paramsConfigs = new FovParamsConfigs();
    paramsConfigs.withFOVRangeX(6.75);
    paramsConfigs.withFOVRangeY(6.75);
    paramsConfigs.withFOVCenterX(6.75);
    paramsConfigs.withFOVCenterY(6.75);

    this.configuration = new CANrangeConfiguration();
    configuration.withFovParams(paramsConfigs);
    rightRange.getConfigurator().apply(configuration);
    leftRange.getConfigurator().apply(configuration);
  }

  @Override
  public void periodic() {}

  public double[] getReferenceRobotPosition(PhotonCamera camera) {
    List<PhotonPipelineResult> results = camera.getAllUnreadResults();

    // Transform Tag Coordinates to Camera Coordinates from photonvision.
    Matrix<N4, N4> transformTagToCamera;

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.getBestTarget() != null && result.getBestTarget().getFiducialId() == 16) {
        // Position of the AprilTag in Robot Coordinates.
        Matrix<N4, N1> referenceRobotPosition = null;

        // Get transformation matrix from photonvision
        transformTagToCamera = result.getBestTarget().getBestCameraToTarget().toMatrix();

        // referenceTagPosition = new Matrix<>(Nat.N4(), Nat.N1(), new double[]{0.381, 0.1524, 0,
        // 1});

        // Transform Tag Position into Robot Coordinates
        referenceRobotPosition =
            VisionConstants.transformFrontLeftToRobot.times(
                transformTagToCamera.times(VisionConstants.referenceTagPosition));
        return referenceRobotPosition.getData();

      } else {
        return new double[] {Double.NaN};
      }

    } else {
      return new double[] {Double.NaN};
    }
  }

  public ChassisSpeeds getAlignSpeeds() {
    double speed;
    double lidarSpeed;
    double gyroSpeed;

    aveLidarDist = (getRightLidarDistance() + getLeftLidarDistance()) / 2;
    diffLidarDist = getRightLidarDistance() - getLeftLidarDistance();
    refPosition = getReferenceRobotPosition(getCamera());

    if (!Double.isNaN(refPosition[0])) {
      speed = pidController.calculate(refPosition[1], 0.1524);
      lidarSpeed =
          areBothLidarsValid()
              ? lidarPIDController.calculate(aveLidarDist, .2)
              : cameraDepthPIDController.calculate(refPosition[0], 0.381);
      gyroSpeed =
          areBothLidarsValid()
              ? gyroPIDController.calculate(Math.asin(diffLidarDist / .605), 0)
              : gyroPIDController.calculate(swerve.getGyro(), Math.toRadians(-60));
    } else {
      speed = 0;
      lidarSpeed = 0;
      gyroSpeed = 0;
    }

    return new ChassisSpeeds(-lidarSpeed, -speed, gyroSpeed);
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
}
