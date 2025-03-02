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
    private VisionSubsystem globalVision;

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

    int[] finalAngles = null;

    boolean xInTolerance = false;
    boolean yInTolerance = false;
    boolean rotInTolerance = false;

    public AlignVision() {

        this.swerve = Swerve.getInstance();
        this.globalVision = VisionSubsystem.getInstance();

        // Choose the correct angles based on the alliance
        this.finalAngles = Constants.isBlueAlliance ? blueAlignAngles : redAlignAngles;

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

        // SmartDashboard.putNumber("AlignVision/PIDS/CameraXPID",
        //         this.cameraXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/CameraYPID",
        //         this.cameraYPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/LidarXPID",
        //         this.lidarXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/LidarRotPID",
        //         this.lidarRotationPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/GyroRotPID",
        //         this.gyroRotationPIDController.getP());

    }

    private PhotonPipelineResult getBestResult(AlignState state) {
        // Gets the best result for each camera from global vision.
        PhotonPipelineResult rightCamResult = globalVision.inputs.frontRightResult;
        PhotonPipelineResult leftCamResult = globalVision.inputs.frontLeftResult;

        // Initialize the best target for each camera to null.
        Transform3d rightBestTransform = null;
        Transform3d leftBestTransform = null;

        // If the state is set of reef or processor, then check which is the best transform
        if (state == AlignState.Reef || state == AlignState.Processor) {
            // If the right camera is not null, has a target and the target is the final tag
            if (rightCamResult != null && rightCamResult.hasTargets() && rightCamResult.getBestTarget().getFiducialId() == finalTagID) {
                rightBestTransform = rightCamResult.getBestTarget().getBestCameraToTarget();
            }

            // If the left camera is not null, has a target and the target is the final tag
            if (leftCamResult != null && leftCamResult.hasTargets() && leftCamResult.getBestTarget().getFiducialId() == finalTagID) {
                leftBestTransform = leftCamResult.getBestTarget().getBestCameraToTarget();
            }

            // If both cameras have a target, then check which is the closer transform
            if (rightBestTransform != null && leftBestTransform != null) {
                // If the right camera is closer to tag, then return the right camera result
                if (rightBestTransform.getTranslation().getDistance(Translation3d.kZero) >= leftBestTransform.getTranslation().getDistance(Translation3d.kZero)) {
                    transformCameraToRobot = VisionConstants.transformFrontLeftToRobot;
                    return leftCamResult;
                } else {
                    transformCameraToRobot = VisionConstants.transformFrontRightToRobot;
                    return rightCamResult;
                }
            } else if (leftBestTransform != null) { // If only the left camera has a target, then return the left camera result
                transformCameraToRobot = VisionConstants.transformFrontLeftToRobot;
                return leftCamResult;
            } else if (rightBestTransform != null) { // If only the right camera has a target, then return the right camera result
                transformCameraToRobot = VisionConstants.transformFrontRightToRobot;
                return rightCamResult;
            } else { // If neither camera has a target, then return null
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
        // Find the Turn Angle for the robot to align with the target.
        turnAngle = handleTurnAngle(state);

        // Get the best result from the global vision
        finalResult = getBestResult(state);

        // this.cameraXPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/CameraXPID",
        //         this.cameraXPIDController.getP()));
        // this.cameraYPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/CameraYPID",
        //         this.cameraYPIDController.getP()));
        // this.lidarXPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/LidarXPID",
        //         this.lidarXPIDController.getP()));
        // this.lidarRotationPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/LidarRotPID",
        //         this.lidarRotationPIDController.getP()));
        // this.gyroRotationPIDController.setP(SmartDashboard.getNumber("AlignVision/PIDS/GyroRotPID",
        //         this.gyroRotationPIDController.getP()));

        // SmartDashboard.putNumber("AlignVision/PIDS/CameraXPID",
        //         this.cameraXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/CameraYPID",
        //         this.cameraYPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/LidarXPID",
        //         this.lidarXPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/LidarRotPID",
        //         this.lidarRotationPIDController.getP());
        // SmartDashboard.putNumber("AlignVision/PIDS/GyroRotPID",
        //         this.gyroRotationPIDController.getP());

        double ySpeed = 0;
        double xSpeed = 0;
        double turnSpeed = 0;
        double targetDistance = 0;
        double aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
        double diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance() - 0.01;
        Pose3d refPosition = null;

        // Calculate the offset index for the reef per poll, side, and orientation
        int currentOffsetIndex = state == AlignState.Reef
                ? calcOrientationOffset(selectedReefOrientation, selectedPoleSide, selectedLevel)
                : 0;

        // If the final result is null, then use global pose to get to the target without hitting the obstacles on the field
        if (finalResult == null) {
            // Get the robot's current position
            Pose3d fieldPosition = new Pose3d(swerve.getPose());

            // Get the tag position from builtin tag layout
            Optional<Pose3d> tagPos = VisionConstants.kTagLayout.getTagPose(finalTagID);

            // If the tag position is present, then get the relative position of the robot to the tag
            if (tagPos.isPresent()) {
                Pose3d tagRelativeToField = fieldPosition.relativeTo(tagPos.get());

                // Makes sure the robot has space to drive to the target, if not don't move robot
                if (tagRelativeToField.getX() > 0.6) {
                    refPosition = tagPos.get().relativeTo(fieldPosition);
                }
            }
        } else { // If the final result is not null, then use the front cameras to get to the target
            refPosition = this.getReferenceRobotPosition(finalResult, transformCameraToRobot);
        }

        // Get the current offset index for the reef calibration offsets
        AlignOffset currentOffset = state == AlignState.Reef ? AlignOffset.values()[currentOffsetIndex] : null;

        try {
            // If the reference position is not null and the turn angle is not the max value, then starts the calculations of the speeds
            if (refPosition != null && turnAngle != Integer.MAX_VALUE) {

                if (Constants.isBlueAlliance && currentOffset != null) { // If the robot is on the blue alliance, then add the blue offset value to the target distance
                    targetDistance += currentOffset.getBlueOffsetValue();
                } else if (!Constants.isBlueAlliance && currentOffset != null) { // If the robot is on the red alliance, then add the red offset value to the target distance
                    targetDistance += currentOffset.getRedOffsetValue();
                }

                if (state == AlignState.Reef) { // If the state is reef, then add the distance to the pole to the target distance
                    if (selectedPoleSide == ReefTargetSide.LEFT) { // If the pole side is left, then add the distance to the pole to the target distance
                        targetDistance += VisionConstants.distanceToPole;
                    } else if (selectedPoleSide == ReefTargetSide.RIGHT) { // If the pole side is right, then subtract the distance to the pole from the target distance
                        targetDistance -= VisionConstants.distanceToPole;
                    }
                } else { // If the state is not reef, then add the distance to the pole to the target distance
                    targetDistance = 0;
                }

                // Check if the robot y position is in tolerance for the target y rotation
                yInTolerance = MathUtil.isNear(-refPosition.getY(), targetDistance, 0.03);

                // Calculate the speeds for the robot to align with the target
                ySpeed = cameraYPIDController.calculate(-refPosition.getY(), targetDistance);

                // Calculate the x and turn speeds for the robot to align with the target
                xSpeed = this.calculateXSpeed(aveLidarDist, refPosition);
                turnSpeed = this.calculateTurnSpeed(diffLidarDist, refPosition);

                // If the turn speed is not a number, then set the x, y, and turn speeds to 0
                if (Double.isNaN(turnSpeed)) {
                    ySpeed = 0;
                    xSpeed = 0;
                    turnSpeed = 0;
                }

            } else { // If the reference position is null or the turn angle is the max value, then set the x, y, and turn speeds to 0
                ySpeed = 0;
                xSpeed = 0;
                turnSpeed = 0;
            }

        } catch (Exception e) { // If everything breaks then set the x, y, and turn speeds to 0
            ySpeed = 0;
            xSpeed = 0;
            turnSpeed = 0;
        }

        // Return the calculated speeds for the robot to align with the target
        return new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
    }

    private int calcOrientationOffset(ReefTargetOrientation orientation, ReefTargetSide side, ElevatorStates level) {

        return (6 * orientation.ordinal()) + (3 * side.ordinal()) + level.ordinal() - 1;
    }

    public int handleTurnAngle(AlignState state) {

        if (state == AlignState.Reef) {

            //Sets the correct tag ID and angles of alignment based on the alliance for Reef
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
            //Sets the correct tag ID and angles of alignment based on the alliance for Processor
            return finalAngles[6];
        } else if (state == AlignState.SourceRight) {
            //Sets the correct tag ID and angles of alignment based on the alliance for SourceRight
            return finalAngles[7];
        } else if (state == AlignState.SourceLeft) {
            //Sets the correct tag ID and angles of alignment based on the alliance for SourceLeft
            return finalAngles[8];
        } else {
            return Integer.MAX_VALUE;
        }
    }

    private double calculateXSpeed(double aveLidarDist, Pose3d refPosition) {
        // If both lidars are valid, then use the lidar distance to calculate the x speed
        if (this.areBothLidarsValid()) {
            // Check if the robot x position is in tolerance for the target x position
            xInTolerance = MathUtil.isNear(aveLidarDist, VisionConstants.maxLidarDepthDistance, 0.03);

            // Calculate the x speed for the robot to align with the target
            return -lidarXPIDController.calculate(aveLidarDist, VisionConstants.maxLidarDepthDistance);
        } else { // If both lidars are not valid, then use the camera distance to calculate the x speed
            // Check if the robot x position is in tolerance for the target x position
            xInTolerance = MathUtil.isNear(refPosition.getX(), VisionConstants.maxCameraDepthDistance, 0.03);

            // Calculate the x speed for the robot to align with the target
            return -cameraXPIDController.calculate(refPosition.getX(), VisionConstants.maxCameraDepthDistance);
        }
    }

    private double calculateTurnSpeed(double diffLidarDist, Pose3d refPosition) {
        // If both lidars are valid, then use the lidar distance to calculate the turn speed
        if (this.areBothLidarsValid()) {
            // Check if the robot rotation is in tolerance for the target rotation
            rotInTolerance = MathUtil.isNear(diffLidarDist, 0, 0.01);

            // Calculate the turn speed for the robot to align with the target
            return -lidarRotationPIDController.calculate(diffLidarDist, 0);
        } else {
            // Check if the robot rotation is in tolerance for the target rotation
            rotInTolerance = MathUtil.isNear(swerve.getPose().getRotation().getRadians(), Math.toRadians(turnAngle), 0.05);

            // Calculate the turn speed for the robot to align with the target
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
