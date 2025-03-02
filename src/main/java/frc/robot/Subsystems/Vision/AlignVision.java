package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
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
import frc.robot.Subsystems.Elevator.ElevatorStates;
import frc.robot.Subsystems.Swerve.Swerve;

import java.util.Optional;

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

    private PhotonPipelineResult finalResult;
    private int finalTagID;
    private double turnAngle;

    private static ReefTargetOrientation selectedReefOrientation = ReefTargetOrientation.AB;
    private static ReefTargetSide selectedPoleSide = ReefTargetSide.RIGHT;
    private static ElevatorStates selectedLevel = ElevatorStates.LEVEL1;
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();
    private Transform3d transformCameraToRobot;

    /*
     * The first six are for the reef, the seventh is for the processor, the eighth
     * is for the right source, and the ninth is for the left source.
     */
    private final int[] blueAlignAngles = { 0, 60, 120, 180, -120, -60, -90, 45, -45 };
    private final int[] redAlignAngles = { 180, 240, 300, 0, 60, 120, 90, -135, 135 };

    boolean xInTolerance = false;
    boolean yInTolerance = false;
    boolean rotInTolerance = false;

    public AlignVision() {

        this.swerve = Swerve.getInstance();
        this.vision = VisionSubsystem.getInstance();

        this.leftRange = new CANrange(13, "KrakensBus");
        this.rightRange = new CANrange(14, "KrakensBus");

        this.cameraXPIDController = new PIDController(0.75, 0, 0);
        this.cameraYPIDController = new PIDController(0.75, 0, 0);
        this.lidarXPIDController = new PIDController(2, 0, 0);
        this.lidarRotationPIDController = new PIDController(8, 0, 0);
        this.gyroRotationPIDController = new PIDController(0.4, 0, 0);
        this.gyroRotationPIDController.enableContinuousInput(0, 2 * Math.PI);

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

        // SmartDashboard.putNumber("AlignVision/CameraXPID",
        // this.cameraXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/CameraYPID",
        // this.cameraYPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/LidarXPID",
        // this.lidarXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/LidarRotPID",
        // this.lidarRotationPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/GyroRotPID",
        // this.gyroRotationPIDController.getP());

    }

    private PhotonPipelineResult getBestResult(AlignState state) {
        PhotonPipelineResult rightCamResult = vision.inputs.frontRightResult;
        PhotonPipelineResult leftCamResult = vision.inputs.frontLeftResult;

        Transform3d rightBestTarget = null;
        Transform3d leftBestTarget = null;

        if (state == AlignState.Reef || state == AlignState.Processor) {
            if (rightCamResult != null && rightCamResult.hasTargets()
                    && rightCamResult.getBestTarget().getFiducialId() == finalTagID) {
                rightBestTarget = rightCamResult.getBestTarget().getBestCameraToTarget();
            }

            if (leftCamResult != null && leftCamResult.hasTargets()
                    && leftCamResult.getBestTarget().getFiducialId() == finalTagID) {
                leftBestTarget = leftCamResult.getBestTarget().getBestCameraToTarget();
            }

            if (rightBestTarget != null && leftBestTarget != null) {
                if (rightBestTarget.getTranslation().getDistance(Translation3d.kZero) >= leftBestTarget.getTranslation().getDistance(Translation3d.kZero)) {
                    transformCameraToRobot = VisionConstants.transformFrontLeftToRobot;
                    return leftCamResult;
                } else {
                    transformCameraToRobot = VisionConstants.transformFrontRightToRobot;
                    return rightCamResult;
                }
            } else if (leftBestTarget != null) {
                transformCameraToRobot = VisionConstants.transformFrontLeftToRobot;
                return leftCamResult;
            } else if (rightBestTarget != null) {
                transformCameraToRobot = VisionConstants.transformFrontRightToRobot;
                return rightCamResult;
            } else {
                return null;
            }

        } else {
            // Change when back camera is added
            transformCameraToRobot = VisionConstants.transformFrontRightToRobot;
            return null;
        }
    }

    private Pose3d getReferenceRobotPosition(PhotonPipelineResult result, Transform3d transformCameraToRobot) {
        // Transform Tag Coordinates to Camera Coordinates from photonvision.
        Transform3d transformTagToCamera;

        if (result != null && result.getBestTarget() != null
                && this.isValidAlignTag(result.getBestTarget().getFiducialId())) {
            // Position of the AprilTag in Robot Coordinates.
            Pose3d referenceRobotPosition;

            // Get transformation matrix from photonvision
            bestTarget = result.getBestTarget();
            transformTagToCamera = bestTarget.getBestCameraToTarget();

            // Transform Tag Position into Robot Coordinates
            referenceRobotPosition = VisionConstants.referenceTagPosition.transformBy(transformTagToCamera.inverse())
                    .transformBy(transformCameraToRobot.inverse());
            return referenceRobotPosition;

        } else {
            return Pose3d.kZero;
        }
    }

    public ChassisSpeeds getAlignChassisSpeeds(AlignState state) {
        turnAngle = handleTurnAngle(state);
        finalResult = getBestResult(state);

        // if (finalResult == null) {
        //     return new ChassisSpeeds(0, 0, 0);
        // }

        // this.cameraXPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/CameraXPID",
        // this.cameraXPIDController.getP()));
        // this.cameraYPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/CameraYPID",
        // this.cameraYPIDController.getP()));
        // this.lidarXPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/LidarXPID",
        // this.lidarXPIDController.getP()));
        // this.lidarRotationPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/LidarRotPID",
        // this.lidarRotationPIDController.getP()));
        // this.gyroRotationPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/GyroRotPID",
        // this.gyroRotationPIDController.getP()));

        // SmartDashboard.putString("Vision Reef Side",
        // AlignVision.selectedPoleSide.name());
        // SmartDashboard.putString("Vision Reef Orientation",
        // AlignVision.selectedReefOrientation.name());

        // SmartDashboard.putNumber("AlignVision/CameraXPID",
        // this.cameraXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/CameraYPID",
        // this.cameraYPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/LidarXPID",
        // this.lidarXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/LidarRotPID",
        // this.lidarRotationPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/GyroRotPID",
        // this.gyroRotationPIDController.getP());

        double ySpeed = 0;
        double xSpeed = 0;
        double turnSpeed = 0;
        double targetDistance = 0;
        double aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
        double diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance() - 0.01;
        Pose3d refPosition;

        int currentOffsetIndex = state == AlignState.Reef
                ? calcOrientationOffset(selectedReefOrientation, selectedPoleSide, selectedLevel)
                : 0;

        SmartDashboard.putNumber("AlignVision/FinalTagID", finalTagID);
        if (finalResult == null) {
            Pose3d fieldPosition = new Pose3d(swerve.getPose());
            Optional<Pose3d> tagPos = VisionConstants.kTagLayout.getTagPose(finalTagID);
            SmartDashboard.putString("AlignVision/TagPose", tagPos.toString());

            if (tagPos.isPresent()) {
                Pose3d tagRelativeToField = fieldPosition.relativeTo(tagPos.get());
                SmartDashboard.putNumber("AlignVision/TagRelFieldX", tagRelativeToField.getX());
                SmartDashboard.putNumber("AlignVision/TagRelFieldY", tagRelativeToField.getY());
                SmartDashboard.putNumber("AlignVision/TagRelFieldZ", tagRelativeToField.getRotation().getZ());

                SmartDashboard.putBoolean("AlignVision/TagGreaterZero", tagRelativeToField.getX() > 0);
                if (tagRelativeToField.getX() > 0) {
                    refPosition = tagPos.get().relativeTo(fieldPosition);
                } else {
                    refPosition = new Pose3d();
                }
            } else {
                refPosition = new Pose3d();
            }
        } else {
            refPosition = this.getReferenceRobotPosition(finalResult, transformCameraToRobot);
        }
        SmartDashboard.putString("AlignVision/Refpos", refPosition.toString());
        AlignOffset currentOffset = state == AlignState.Reef ? AlignOffset.values()[currentOffsetIndex] : null;

        try {
            if (refPosition.getX() != Transform2d.kZero.getX()
                    && refPosition.getY() != Transform2d.kZero.getY() && !Double.isNaN(turnAngle)) {

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

                ySpeed = cameraYPIDController.calculate(-refPosition.getY(), targetDistance);
                yInTolerance = MathUtil.isNear(-refPosition.getY(), targetDistance, 0.03);
                xSpeed = this.calculateXSpeed(aveLidarDist, refPosition);
                SmartDashboard.putNumber("AlignVision/TargetDist", targetDistance);

                SmartDashboard.putBoolean("AlignVision/XTolerance", xInTolerance);
                SmartDashboard.putBoolean("AlignVision/YTolerance", yInTolerance);
                SmartDashboard.putBoolean("AlignVision/RotTolerance", rotInTolerance);

                turnSpeed = this.calculateTurnSpeed(diffLidarDist, refPosition);

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

    private int calcOrientationOffset(ReefTargetOrientation orientation, ReefTargetSide side,
            ElevatorStates level) {

        return (6 * orientation.ordinal()) + (3 * side.ordinal()) + level.ordinal() - 1;
    }

    public int handleTurnAngle(AlignState state) {
        // SmartDashboard.putString("AlignVision/State", state.toString());
        // Different orientations of the reef (Degrees)

        int[] finalAngles = Constants.isBlueAlliance ? blueAlignAngles : redAlignAngles;

        if (state == AlignState.Reef) {
            switch (selectedReefOrientation) {
                case AB:
                    finalTagID = Constants.isBlueAlliance ? 18 : 7;
                    return finalAngles[0];
                case CD:
                    finalTagID = Constants.isBlueAlliance ? 17 : 8;
                    return finalAngles[1];
                case EF:
                    finalTagID = Constants.isBlueAlliance ? 22 : 9;
                    return finalAngles[2];
                case GH:
                    finalTagID = Constants.isBlueAlliance ? 21 : 10;
                    return finalAngles[3];
                case IJ:
                    finalTagID = Constants.isBlueAlliance ? 20 : 11;
                    return finalAngles[4];
                case KL:
                    finalTagID = Constants.isBlueAlliance ? 19 : 6;
                    return finalAngles[5];
                default:
                    return Integer.MAX_VALUE;
            }
        } else if (state == AlignState.Processor) {
            return finalAngles[6];
        } else if (state == AlignState.SourceRight) {
            return finalAngles[7];
        } else if (state == AlignState.SourceLeft) {
            return finalAngles[8];
        } else {
            return Integer.MAX_VALUE;
        }
    }

    private double calculateXSpeed(double aveLidarDist, Pose3d refPosition) {
        if (this.areBothLidarsValid()) {
            xInTolerance = MathUtil.isNear(aveLidarDist, VisionConstants.maxLidarDepthDistance, 0.03);

            return -lidarXPIDController.calculate(aveLidarDist,
                    VisionConstants.maxLidarDepthDistance);
        } else {
            xInTolerance = MathUtil.isNear(refPosition.getX(),
                    VisionConstants.maxCameraDepthDistance, 0.03);

            return -cameraXPIDController.calculate(refPosition.getX(),
                    VisionConstants.maxCameraDepthDistance);
        }
    }

    private double calculateTurnSpeed(double diffLidarDist, Pose3d refPosition) {
        if (turnAngle == Integer.MAX_VALUE) {
            return Double.NaN;
        }
        if (this.areBothLidarsValid()) {
            rotInTolerance = MathUtil.isNear(diffLidarDist, 0, 0.01);

            return -lidarRotationPIDController.calculate(diffLidarDist, 0);
        } else {
            rotInTolerance = MathUtil.isNear(swerve.getPose().getRotation().getRadians(), Math.toRadians(turnAngle), 0.05);

            return gyroRotationPIDController.calculate(swerve.getPose().getRotation().getRadians(), Math.toRadians(turnAngle));
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

    public static void setPoleLevel(ElevatorStates level) {
        SmartDashboard.putString("level", level.name());
        selectedLevel = level;
    }

    public boolean isAligned() {
        return xInTolerance && yInTolerance && rotInTolerance;
    }
}
