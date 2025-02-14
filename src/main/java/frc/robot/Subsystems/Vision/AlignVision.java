package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.States.AlignOffset;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
// import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import frc.robot.Subsystems.Swerve.Swerve;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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

  private PIDController cameraYPIDController;
  private PIDController lidarXPIDController;
  private PIDController cameraXPIDController;
  private PIDController gyroRotationPIDController;
  private PIDController lidarRotationPIDController;

  private PhotonPipelineResult result;

  private double turnAngle;
  private static ReefTargetOrientation selectedReefOrientation = null;
  private static ReefTargetSide selectedPoleSide = null;
  private static ReefTargetLevel selectedLevel = null;
  private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();

  boolean xInTolerance = false;
  boolean yInTolerance = false;
  boolean rotInTolerance = false;

  public AlignVision() {

    this.swerve = Swerve.getInstance();
    this.vision = VisionSubsystem.getInstance();

    this.leftRange = new CANrange(13, "KrakensBus");
    this.rightRange = new CANrange(14, "KrakensBus");

    this.cameraXPIDController = new PIDController(4, 0, 0);
    this.cameraYPIDController = new PIDController(4, 0, 0);
    this.lidarXPIDController = new PIDController(4, 0, 0);
    this.gyroRotationPIDController = new PIDController(0.2, 0, 0);
    this.gyroRotationPIDController.enableContinuousInput(0, 2 * Math.PI);
    this.lidarRotationPIDController = new PIDController(10, 0, 0);

    this.paramsConfigs = new FovParamsConfigs();
    paramsConfigs.withFOVRangeX(6.75);
    paramsConfigs.withFOVRangeY(6.75);
    paramsConfigs.withFOVCenterX(6.75);
    paramsConfigs.withFOVCenterY(6.75);

    this.configuration = new CANrangeConfiguration();
    configuration.withFovParams(paramsConfigs);
    configuration.ProximityParams.ProximityThreshold = 1;
    rightRange.getConfigurator().apply(configuration);
    leftRange.getConfigurator().apply(configuration);
  }

  // private Pose3d getReferenceRobotPosition() {

  // // Transform Tag Coordinates to Camera Coordinates from photonvision.
  // Matrix<N4, N4> transformTagToCamera;

  // if (result != null) {
  // if (result.getBestTarget() != null
  // && this.isValidAlignTag(result.getBestTarget().getFiducialId())) {
  // // Position of the AprilTag in Robot Coordinates.
  // Matrix<N4, N1> referenceRobotPosition = null;

  // // Get transformation matrix from photonvision
  // transformTagToCamera =
  // result.getBestTarget().getBestCameraToTarget().toMatrix();

  // // referenceTagPosition = new Matrix<>(Nat.N4(), Nat.N1(), new
  // double[]{0.381,
  // // 0.1524, 0,
  // // 1});

  // // Transform Tag Position into Robot Coordinates
  // referenceRobotPosition =
  // VisionConstants.transformFrontLeftToRobot
  // .toMatrix()
  // .times(
  // transformTagToCamera.times(
  // new Matrix<>(Nat.N4(), Nat.N1(), new double[] {0, 0, 0, 1})));

  // return new Pose3d(
  // new Translation3d(
  // referenceRobotPosition.getData()[0], referenceRobotPosition.getData()[1], 0),
  // new Rotation3d());

  // } else {
  // return Pose3d.kZero;
  // }

  // } else {
  // return Pose3d.kZero;
  // }
  // }

  private Pose3d getReferenceRobotPosition() {
    // Transform Tag Coordinates to Camera Coordinates from photonvision.
    Transform3d transformTagToCamera;

    if (result != null
        && result.getBestTarget() != null
        && this.isValidAlignTag(result.getBestTarget().getFiducialId())) {
      // Position of the AprilTag in Robot Coordinates.
      Pose3d referenceRobotPosition;

      // Get transformation matrix from photonvision
      bestTarget = result.getBestTarget();
      transformTagToCamera = bestTarget.getBestCameraToTarget(); // might need to invert

      // Transform Tag Position into Robot Coordinates
      referenceRobotPosition = VisionConstants.referenceTagPosition
          .transformBy(transformTagToCamera.inverse())
          .transformBy(VisionConstants.transformFrontLeftToRobot.inverse());
      return referenceRobotPosition;

    } else {
      return Pose3d.kZero;
    }
  }

  public ChassisSpeeds getAlignChassisSpeeds(AlignState state) {
    result = vision.inputs.frontLeftResult;
    double ySpeed = 0;
    double xSpeed = 0;
    double turnSpeed = 0;
    double targetDistance = 0;
    double aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
    double diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance() - 0.01;

    int currentOffsetIndex = state == AlignState.Reef
        ? calcOrientationOffset(selectedReefOrientation, selectedPoleSide, selectedLevel)
        : 0;

    Pose3d refPosition = this.getReferenceRobotPosition();
    SmartDashboard.putString("Refpos", refPosition.toString());
    AlignOffset currentOffset = state == AlignState.Reef ? AlignOffset.values()[currentOffsetIndex] : null;

    SmartDashboard.putBoolean("is both lidar", areBothLidarsValid());

    try {
      if (refPosition.getX() != Transform2d.kZero.getX()
          && refPosition.getY() != Transform2d.kZero.getY()
          && !Double.isNaN(turnAngle)) {

        if (Constants.isBlueAlliance && currentOffset != null) {
          targetDistance += currentOffset.getBlueOffsetValue();
        } else if (!Constants.isBlueAlliance && currentOffset != null) {
          targetDistance += currentOffset.getRedOffsetValue();
        }

        if (state == AlignState.Reef) {
          if (selectedPoleSide == ReefTargetSide.LEFT) {
            targetDistance += VisionConstants.distanceToPole;
          } else if (selectedPoleSide == ReefTargetSide.RIGHT) {
            targetDistance -= VisionConstants.distanceToPole;
          }
        } else {
          targetDistance = 0;
        }

        SmartDashboard.putNumber("RefPoseY", refPosition.getY());
        SmartDashboard.putNumber("RefPoseX", refPosition.getX());

        ySpeed = cameraYPIDController.calculate(-refPosition.getY(), targetDistance);
        yInTolerance = MathUtil.isNear(-refPosition.getY(), targetDistance, 0.05);
        xSpeed = this.calculateXSpeed(aveLidarDist, refPosition);
        SmartDashboard.putNumber("TargetDist", targetDistance);

        turnSpeed = this.calculateTurnSpeed(diffLidarDist, refPosition, state);

        if (Double.isNaN(turnSpeed)) {
          ySpeed = 0;
          xSpeed = 0;
          turnSpeed = 0;
        }

      } else {
        ySpeed = 0;
        xSpeed = 0;
        turnSpeed = 0;
      }
      SmartDashboard.putNumber("rot", bestTarget.getYaw());

    } catch (Exception e) {
      ySpeed = 0;
      xSpeed = 0;
      turnSpeed = 0;
    }

    return new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
  }

  private int calcOrientationOffset(
      ReefTargetOrientation orientation, ReefTargetSide side, ReefTargetLevel level) {

    return (6 * orientation.ordinal()) + (3 * side.ordinal()) + level.ordinal();
  }

  private int handleTurnAngle(AlignState state) {
    // Different orientations of the reef (Degrees)
    if (state == AlignState.Reef) {
      switch (selectedReefOrientation) {
        case AB:
          return 0;
        case CD:
          return 60;
        case EF:
          return 120;
        case GH:
          return 180;
        case IJ:
          return -120;
        case KL:
          return 60;
        default:
          return Integer.MAX_VALUE;
      }
    } else if (state == AlignState.Processor) {
      return -90;
    } else if (state == AlignState.Source
        && (this.isValidAlignTag(1) || this.isValidAlignTag(13))) {
      return 135;
    } else if (state == AlignState.Source
        && (this.isValidAlignTag(2) || this.isValidAlignTag(12))) {
      return -135;
    } else {
      return Integer.MAX_VALUE;
    }
  }

  private double calculateXSpeed(double aveLidarDist, Pose3d refPosition) {
    SmartDashboard.putNumber("avg lidar dist", aveLidarDist);
    SmartDashboard.putNumber("camerax", refPosition.getX());
    if (this.areBothLidarsValid()) {
      xInTolerance = MathUtil.isNear(aveLidarDist, VisionConstants.maxLidarDepthDistance, 0.05);

      return -lidarXPIDController.calculate(aveLidarDist, VisionConstants.maxLidarDepthDistance);
    } else {
      xInTolerance = MathUtil.isNear(refPosition.getX(), VisionConstants.maxCameraDepthDistance, 0.05);

      return -cameraXPIDController.calculate(
          refPosition.getX(), VisionConstants.maxCameraDepthDistance);
    }
  }

  private double calculateTurnSpeed(double diffLidarDist, Pose3d refPosition, AlignState state) {
    turnAngle = handleTurnAngle(state);

    if (turnAngle == Integer.MAX_VALUE) {
      return Double.NaN;
    }
    if (this.areBothLidarsValid()) {
      rotInTolerance = MathUtil.isNear(diffLidarDist, 0, 0.01);

      return -lidarRotationPIDController.calculate(diffLidarDist, 0);
    } else {
      rotInTolerance = MathUtil.isNear(swerve.getGyro(), Math.toRadians(turnAngle), 0.1);

      return -gyroRotationPIDController.calculate(swerve.getGyro(), Math.toRadians(turnAngle));
    }

  }

  private boolean isValidAlignTag(int tagID) {
    return VisionConstants.validAlignTags.contains(tagID);
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
    SmartDashboard.putString("Orientation", orientation.name());
    selectedReefOrientation = orientation;
  }

  public static void setPoleSide(ReefTargetSide side) {
    SmartDashboard.putString("Side", side.name());
    selectedPoleSide = side;
  }

  public static void setPoleLevel(ReefTargetLevel level) {
    SmartDashboard.putString("level", level.name());
    selectedLevel = level;
  }

  public boolean isAligned() {
    return xInTolerance && yInTolerance && rotInTolerance;
  }
}
