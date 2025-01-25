package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.States.AlignOffset;
import frc.robot.States.ReefTargetLevel;
// import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
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

  private List<PhotonPipelineResult> results;
  private double aveLidarDist;
  private double diffLidarDist;
  private Transform2d refPosition;

  private double ySpeed;
  private double xSpeed;
  private double turnSpeed;
  private int currentOffset;
  // private static ReefTargetOrientation selectedReefOrientation;
  private static ReefTargetSide selectedPoleSide;
  private static ReefTargetLevel selectedLevel;

  public AlignVision() {
    this.swerve = Swerve.getInstance();

    this.cam = new PhotonCamera("CamOne");
    this.leftRange = new CANrange(10);
    this.rightRange = new CANrange(12);

    this.pidController = new PIDController(2, 0, 0);
    this.lidarPIDController = new PIDController(2, 0, 0);
    this.cameraDepthPIDController = new PIDController(1.25, 0, 0);
    this.gyroPIDController = new PIDController(4, 0, 0);
    this.gyroPIDController.enableContinuousInput(0, 2 * Math.PI);

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
  public void periodic() {
    results = cam.getAllUnreadResults();
  }

  public Transform2d getReferenceRobotPosition() {
    // Transform Tag Coordinates to Camera Coordinates from photonvision.
    Matrix<N4, N4> transformTagToCamera;

    if (!results.isEmpty()) {
      var result = results.get(results.size() - 1);

      if (result.getBestTarget() != null && result.getBestTarget().getFiducialId() == 16) {
        // Position of the AprilTag in Robot Coordinates.
        Matrix<N4, N1> referenceRobotPosition = null;

        // Get transformation matrix from photonvision
        transformTagToCamera = result.getBestTarget().getBestCameraToTarget().toMatrix();

        // referenceTagPosition = new Matrix<>(Nat.N4(), Nat.N1(), new double[]{0.381,
        // 0.1524, 0,
        // 1});

        // Transform Tag Position into Robot Coordinates
        referenceRobotPosition = VisionConstants.transformFrontLeftToRobot.times(
            transformTagToCamera.times(VisionConstants.referenceTagPosition));
        return new Transform2d(
            referenceRobotPosition.getData()[0],
            referenceRobotPosition.getData()[1],
            new Rotation2d(referenceRobotPosition.getData()[2]));

      } else {
        return Transform2d.kZero;
      }

    } else {
      return Transform2d.kZero;
    }
  }

  // private double handleAngle() {
  //   if (selectedReefOrientation == ReefTargetOrientation.AB) {
  //     return 0;
  //   } else if (selectedReefOrientation == ReefTargetOrientation.CD) {
  //     return 60;
  //   } else if (selectedReefOrientation == ReefTargetOrientation.EF) {
  //     return 120;
  //   } else if (selectedReefOrientation == ReefTargetOrientation.GH) {
  //     return 180;
  //   } else if (selectedReefOrientation == ReefTargetOrientation.IJ) {
  //     return -120;
  //   } else if (selectedReefOrientation == ReefTargetOrientation.KL) {
  //     return -60;
  //   } else {
  //     return Double.NaN;
  //   }
  // }

  private int calcFindOffset(int tagID, ReefTargetSide side, ReefTargetLevel level) {
    int offset = Constants.isBlueAlliance ? 17 : 6;

    return 6 * (tagID - offset) + (3 * side.ordinal()) + level.ordinal();
  }

  public ChassisSpeeds getAlignChassisSpeeds() {
    aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
    diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance();
    refPosition = this.getReferenceRobotPosition();
    currentOffset = calcFindOffset(currentOffset, selectedPoleSide, selectedLevel);

    try {
      if (refPosition.getX() != Transform2d.kZero.getX()
          && refPosition.getY() != Transform2d.kZero.getY() && !Double.isNaN(0)) {

        ySpeed = pidController.calculate(refPosition.getY(), 0.1524);
        xSpeed = this.areBothLidarsValid()
            ? lidarPIDController.calculate(aveLidarDist, .12)
            : cameraDepthPIDController.calculate(refPosition.getX(), 0.381);
        turnSpeed = this.areBothLidarsValid()
            ? -gyroPIDController.calculate(Math.asin(diffLidarDist / .605), 0)
            : gyroPIDController.calculate(swerve.getGyro(), Math.toRadians(0));

      } else {
        ySpeed = 0;
        xSpeed = 0;
        turnSpeed = 0;
      }
    } catch (Exception e) {
      ySpeed = 0;
      xSpeed = 0;
      turnSpeed = 0;
    }

    return new ChassisSpeeds(-xSpeed, -ySpeed, turnSpeed);
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

  // public static void setReefOrientation(ReefTargetOrientation orientation) {
  //   selectedReefOrientation = orientation;
  // }

  public static void setPoleSide(ReefTargetSide side) {
    selectedPoleSide = side;
  }

  public static void setPoleLevel(ReefTargetLevel level) {
    selectedLevel = level;
  }
}
