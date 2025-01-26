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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.States.AlignOffset;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
// import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Swerve.Swerve;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
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
  private VisionSubsystem vision;

  private CANrangeConfiguration configuration;
  private FovParamsConfigs paramsConfigs;
  private CANrange rightRange;
  private CANrange leftRange;

  private PIDController pidController;
  private PIDController lidarPIDController;
  private PIDController cameraDepthPIDController;
  private PIDController gyroPIDController;

  private PhotonPipelineResult result;
  private double aveLidarDist;
  private double diffLidarDist;
  private Transform2d refPosition;

  private double ySpeed;
  private double xSpeed;
  private double turnSpeed;
  private double turnAngle;
  private int currentOffsetIndex;
  private AlignOffset currentOffset;
  private static ReefTargetOrientation selectedReefOrientation;
  private static ReefTargetSide selectedPoleSide;
  private static ReefTargetLevel selectedLevel;

  public AlignVision() {
    this.swerve = Swerve.getInstance();
    this.vision = VisionSubsystem.getInstance();

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

  private Transform2d getReferenceRobotPosition() {
    // Transform Tag Coordinates to Camera Coordinates from photonvision.
    Matrix<N4, N4> transformTagToCamera;

    if (result != null) {
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

  private int calcFindOffset(ReefTargetOrientation orientation, ReefTargetSide side, ReefTargetLevel level) {

    if (orientation == ReefTargetOrientation.AB) {
      turnAngle = 0;
    } else if (orientation == ReefTargetOrientation.CD) {
      turnAngle = 60;
    } else if (orientation == ReefTargetOrientation.EF) {
      turnAngle = 120;
    } else if (orientation == ReefTargetOrientation.GH) {
      turnAngle = 180;
    } else if (orientation == ReefTargetOrientation.IJ) {
      turnAngle = -120;
    } else if (orientation == ReefTargetOrientation.KL) {
      turnAngle = -60;
    } else {
      turnAngle = Double.NaN;
    }

    return (6 * orientation.ordinal()) + (3 * side.ordinal()) + level.ordinal();
  }

  public ChassisSpeeds getAlignChassisSpeeds() {
    result = vision.inputs.frontLeftResult;

    aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
    diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance();
    refPosition = this.getReferenceRobotPosition();
    currentOffsetIndex = calcFindOffset(selectedReefOrientation, selectedPoleSide, selectedLevel);
    currentOffset = AlignOffset.values()[currentOffsetIndex];
    SmartDashboard.putNumber("currentOffsetIndex", currentOffsetIndex);
    SmartDashboard.putNumber("currentOffset", currentOffset.getBlueOffsetValue());
    var target = 0;

    try {
      if (refPosition.getX() != Transform2d.kZero.getX()
          && refPosition.getY() != Transform2d.kZero.getY()
          && !Double.isNaN(turnAngle)) {

        if (Constants.isBlueAlliance) {
          target += currentOffset.getBlueOffsetValue();
        } else {
          target += currentOffset.getRedOffsetValue();
        }

        if (selectedPoleSide == ReefTargetSide.LEFT) {
          target += VisionConstants.distanceToPole;
        } else {
          target -= VisionConstants.distanceToPole;
        }

        ySpeed = pidController.calculate(refPosition.getY(), target);
        xSpeed = this.areBothLidarsValid()
            ? lidarPIDController.calculate(aveLidarDist, .12)
            : cameraDepthPIDController.calculate(refPosition.getX(), 0.381);

        turnSpeed = this.areBothLidarsValid()
            ? -gyroPIDController.calculate(Math.asin(diffLidarDist / .605), 0)
            : gyroPIDController.calculate(swerve.getGyro(), Math.toRadians(turnAngle));

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

  public static void setReefOrientation(ReefTargetOrientation orientation) {
    selectedReefOrientation = orientation;
  }

  public static void setPoleSide(ReefTargetSide side) {
    selectedPoleSide = side;
  }

  public static void setPoleLevel(ReefTargetLevel level) {
    selectedLevel = level;
  }
}
