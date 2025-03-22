package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
import frc.robot.Subsystems.Lights.TimedAnimation;
import frc.robot.Subsystems.Lights.Light;
import frc.robot.Subsystems.Lights.LightsIO;
import frc.robot.Subsystems.Swerve.Swerve;
import frc.robot.Subsystems.Swerve.Swerve.DriveState;

import java.io.Console;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;
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
    private CANrange backRange;

    private PIDController cameraXPIDController;
    private PIDController backCameraXPIDController;
    private PIDController cameraYPIDController;
    private PIDController cameraYPIDControllerSource;

    private PIDController lidarXPIDController;
    private PIDController backLidarXPIDController;

    private PIDController gyroRotationPIDController;
    private PIDController lidarRotationPIDController;

    private final TrapezoidProfile yProfile = new TrapezoidProfile(new TrapezoidProfile.Constraints(5, 2));

    private TrapezoidProfile.State yGoal = new TrapezoidProfile.State();
    public static TrapezoidProfile.State ySetpoint = new TrapezoidProfile.State();

    private boolean isFirstTime = false;
    private boolean usingGlobalVision = false;

    private double deltaTime = 0.02;

    private PhotonPipelineResult finalResult;
    private int finalTagID;
    private double turnAngle;

    private static ReefTargetOrientation selectedReefOrientation = ReefTargetOrientation.AB;
    private static ReefTargetSide selectedPoleSide = ReefTargetSide.RIGHT;
    private static ElevatorStates selectedLevel = ElevatorStates.LEVEL1;
    public static boolean isCollecting = false;
    private PhotonTrackedTarget bestTarget = new PhotonTrackedTarget();
    private Transform3d transformCameraToRobot;

    private double distanceToLeftSourceSide;
    private double distanceToRightSourceSide;

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

        this.leftRange = new CANrange(13, "KrakensBus");
        this.rightRange = new CANrange(14, "KrakensBus");
        this.backRange = new CANrange(23, "KrakensBus");

        this.cameraXPIDController = new PIDController(2, 0, 0);
        this.backCameraXPIDController = new PIDController(3.5, 0, 0);

        this.cameraYPIDController = new PIDController(2.75, 0, 0);
        this.cameraYPIDControllerSource = new PIDController(2, 0, 0);

        this.lidarXPIDController = new PIDController(4, 0, 0);
        this.backLidarXPIDController = new PIDController(5.5, 0, 0);

        this.lidarRotationPIDController = new PIDController(10, 0, 0);
        this.gyroRotationPIDController = new PIDController(4, 0, 0);
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
        backRange.getConfigurator().apply(configuration);

    }

    public boolean TagIsInView(int targetTagID) {
        // get the tag id of the best result for both camera
        PhotonPipelineResult leftCamera = globalVision.inputs.frontLeftResult;
        PhotonPipelineResult rightCamera = globalVision.inputs.frontRightResult;
        // if either camera can see the target tag return true
        if (rightCamera != null && rightCamera.hasTargets() && rightCamera.getBestTarget().getFiducialId() == targetTagID ||
                leftCamera != null && leftCamera.hasTargets() && leftCamera.getBestTarget().getFiducialId() == targetTagID) {
            return true;
        }
        // otherwise return false
        return false;
    }

    private PhotonPipelineResult getBestResult(AlignState state) {
        // Gets the best result for each camera from global vision.
        PhotonPipelineResult rightCamResult = globalVision.inputs.frontRightResult;
        PhotonPipelineResult leftCamResult = globalVision.inputs.frontLeftResult;
        PhotonPipelineResult backCamResult = globalVision.inputs.backMiddleResult;

        // rightCamResult.getBestTarget().toString();
        // leftCamResult.getBestTarget().toString();
        // backCamResult.getBestTarget();

        // Initialize the best target for each camera to null.
        Transform3d rightBestTransform = null;
        Transform3d leftBestTransform = null;

        // If the state is set of reef or processor, then check which is the best
        // transform
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

        } else if (backCamResult != null && backCamResult.hasTargets() && backCamResult.getBestTarget().getFiducialId() == finalTagID
                && (state == AlignState.SourceLeft || state == AlignState.SourceRight)) {
            // If the back camera is not null, has a target and the target is the final tag
            transformCameraToRobot = VisionConstants.transformBackToRobot;
            return backCamResult;
        } else {
            transformCameraToRobot = VisionConstants.transformBackToRobot;
            return null;
        }
    }

    private Pose3d getReferenceRobotPosition(PhotonPipelineResult result, Transform3d transformCameraToRobot) {
        // Transform Tag Coordinates to Camera Coordinates from photonvision.
        Transform3d transformCameraToTag;

        if (result != null && result.getBestTarget() != null
                && this.isValidAlignTag(result.getBestTarget().getFiducialId())) {
            // Position of the AprilTag in Robot Coordinates.
            Matrix<N4, N1> referenceRobotPosition;

            // Get transformation matrix from photonvision
            bestTarget = result.getBestTarget();
            transformCameraToTag = bestTarget.getBestCameraToTarget();

            // Transform Tag Position into Robot Coordinates
            // referenceRobotPosition =
            // VisionConstants.referenceTagPosition.transformBy(transformCameraToTag);
            referenceRobotPosition = transformCameraToTag.toMatrix().times(VisionConstants.referenceTagPosition);
            // referenceRobotPosition =
            // referenceRobotPosition.transformBy(transformCameraToRobot);
            referenceRobotPosition = transformCameraToRobot.toMatrix().times(referenceRobotPosition);

            return new Pose3d(referenceRobotPosition.get(0, 0), referenceRobotPosition.get(1, 0), referenceRobotPosition.get(2, 0),
                    new Rotation3d());

        } else {
            return Pose3d.kZero;
        }
    }

    public ChassisSpeeds getAlignChassisSpeeds(AlignState state) {
        try {
            this.finalAngles = Constants.isBlueAlliance ? blueAlignAngles : redAlignAngles;

            if (swerve.getPreviousDriveState() != DriveState.AlignReef || swerve.getPreviousDriveState() != DriveState.AlignSource
                    || swerve.getPreviousDriveState() != DriveState.AlignProcessor) {
                isFirstTime = true;

                cameraXPIDController.reset();
                cameraYPIDController.reset();
                cameraYPIDControllerSource.reset();
                backCameraXPIDController.reset();
                lidarXPIDController.reset();
                backLidarXPIDController.reset();
                lidarRotationPIDController.reset();
                gyroRotationPIDController.reset();

            } else {
                isFirstTime = false;
            }

            // Find the Turn Angle for the robot to align with the target.
            turnAngle = handleTurnAngle(state);

            // Get the best result from the global vision
            finalResult = getBestResult(state);

            double ySpeed = 0;
            double xSpeed = 0;
            double turnSpeed = 0;
            double targetDistance = 0;
            double aveLidarDist = (this.getRightLidarDistance() + this.getLeftLidarDistance()) / 2;
            double diffLidarDist = this.getRightLidarDistance() - this.getLeftLidarDistance() - 0.01;
            Pose3d refPosition = null;

            // If the final result is null, then use global pose to get to the target
            // without hitting the obstacles on the field
            if (finalResult == null) {
                usingGlobalVision = true;
                // Get the robot's current position
                Pose3d fieldPosition = new Pose3d(swerve.getPose());

                // Get the tag position from builtin tag layout
                Optional<Pose3d> tagPos = VisionConstants.kTagLayout.getTagPose(finalTagID);

                // If the tag position is present, then get the relative position of the robot
                // to the tag
                if (tagPos.isPresent()) {
                    Pose3d tagRelativeToField = fieldPosition.relativeTo(tagPos.get());

                    // Makes sure the robot has space to drive to the target, if not don't move
                    // robot
                    if (tagRelativeToField.getX() > 0.3) {
                        refPosition = tagPos.get().relativeTo(fieldPosition);
                    }
                }
            } else { // If the final result is not null, then use the front cameras to get to the
                     // target
                usingGlobalVision = false;
                refPosition = this.getReferenceRobotPosition(finalResult, transformCameraToRobot);
            }

            try {
                // If the reference position is not null and the turn angle is not the max
                // value, then starts the calculations of the speeds
                if (refPosition != null && turnAngle != Integer.MAX_VALUE) {

                    if (state == AlignState.Reef) { // If the state is reef, then add the distance to the pole to the target
                                                    // distance
                        if (selectedPoleSide == ReefTargetSide.LEFT) { // If the pole side is left, then add the distance to the pole to the target
                                                                       // distance
                            targetDistance -= VisionConstants.distanceToPole;
                        } else if (selectedPoleSide == ReefTargetSide.RIGHT) { // If the pole side is right, then subtract the distance to the pole from the
                                                                               // target distance
                            targetDistance += VisionConstants.distanceToPole;
                        } else {
                            targetDistance = 0;
                        }

                    } else { // If the state is not reef, then add the distance to the pole to the target
                             // distance
                        targetDistance = 0;

                    }

                    if (isFirstTime) { // If it is the first time, then set the x, y, and turn speeds to 0
                        ySetpoint = new TrapezoidProfile.State(refPosition.getY(), 0);
                        yGoal = new TrapezoidProfile.State(targetDistance, 0);
                    }

                    // Check if the robot y position is in tolerance for the target y rotation
                    SmartDashboard.putNumber("Vision/RefX", refPosition.getX());
                    SmartDashboard.putNumber("Vision/RefY", refPosition.getY());
                    SmartDashboard.putNumber("Vision/TargetY", targetDistance);
                    yInTolerance = MathUtil.isNear(refPosition.getY(), targetDistance, 0.03);
                    ySetpoint = yProfile.calculate(deltaTime, ySetpoint, yGoal);
                    // Logger.recordOutput("Vision/SetPointY", ySetpoint.position);

                    // Calculate the speeds for the robot to align with the target
                    ySpeed = ((state == AlignState.SourceLeft) || state == AlignState.SourceRight) ? -cameraYPIDControllerSource.calculate(refPosition.getY(), targetDistance)
                            : -cameraYPIDController.calculate(refPosition.getY(), targetDistance);

                    SmartDashboard.putBoolean("AlignVision/UsingGlobalVision", usingGlobalVision);

                    xSpeed = this.calculateXSpeed(aveLidarDist, refPosition, state);

                    // Calculate the x and turn speeds for the robot to align with the target
                    turnSpeed = this.calculateTurnSpeed(diffLidarDist, refPosition);

                    // If the turn speed is not a number, then set the x, y, and turn speeds to 0
                    if (Double.isNaN(turnSpeed)) {
                        ySpeed = 0;
                        xSpeed = 0;
                        turnSpeed = 0;
                    }

                } else { // If the reference position is null or the turn angle is the max value, then
                         // set the x, y, and turn speeds to 0
                    ySpeed = 0;
                    xSpeed = 0;
                    turnSpeed = 0;
                }

            } catch (Exception e) { // If everything breaks then set the x, y, and turn speeds to 0
                ySpeed = 0;
                xSpeed = 0;
                turnSpeed = 0;
            }

            SmartDashboard.putNumber("AlignVision/XSpeed", xSpeed);
            SmartDashboard.putNumber("AlignVision/YSpeed", ySpeed);
            SmartDashboard.putNumber("AlignVision/TurnSpeed", turnSpeed);

            // Return the calculated speeds for the robot to align with the target
            return new ChassisSpeeds(xSpeed, ySpeed, turnSpeed);
        } catch (Exception exception) {
            System.out.println(exception.toString());
            return new ChassisSpeeds();
        }
    }

    public void resetTolerances() {
        xInTolerance = false;
        yInTolerance = false;
        rotInTolerance = false;
    }

    public int handleTurnAngle(AlignState state) {
        SmartDashboard.putString("selectedReefOrientation", selectedReefOrientation.toString());
        if (state == AlignState.Reef) {

            // Sets the correct tag ID and angles of alignment based on the alliance for
            // Reef
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
            // Sets the correct tag ID and angles of alignment based on the alliance for
            // Processor
            return finalAngles[6];
        } else if (state == AlignState.SourceRight) {
            finalTagID = Constants.isBlueAlliance ? 12 : 2;
            // Sets the correct tag ID and angles of alignment based on the alliance for
            // SourceRight
            return finalAngles[7];
        } else if (state == AlignState.SourceLeft) {
            finalTagID = Constants.isBlueAlliance ? 13 : 1;
            // Sets the correct tag ID and angles of alignment based on the alliance for
            // SourceLeft
            return finalAngles[8];
        } else {
            return Integer.MAX_VALUE;
        }
    }

    private double calculateXSpeed(double aveLidarDist, Pose3d refPosition, AlignState state) {
        // If both lidars are valid, then use the lidar distance to calculate the x
        // speed
        if (state == AlignState.Reef && this.areBothLidarsValid()) {
            isCollecting = false;

            // Check if the robot x position is in tolerance for the target x position
            xInTolerance = MathUtil.isNear(aveLidarDist, VisionConstants.maxFrontLidarDepthDistance, 0.03);

            // Calculate the x speed for the robot to align with the target
            return -lidarXPIDController.calculate(aveLidarDist, VisionConstants.maxFrontLidarDepthDistance);

        } else if ((state == AlignState.SourceLeft || state == AlignState.SourceRight)) {
            if (this.getBackLidarDetect()) {
                isCollecting = true;

                // Check if the robot x position is in tolerance for the target x position
                xInTolerance = MathUtil.isNear(this.getBackLidarDistance(), VisionConstants.maxBackLidarDepthDistance, 0.03);

                // Calculate the x speed for the robot to align with the target
                return backLidarXPIDController.calculate(this.getBackLidarDistance(), VisionConstants.maxBackLidarDepthDistance);

            } else {
                isCollecting = false;

                // Check if the robot x position is in tolerance for the target x position
                xInTolerance = MathUtil.isNear(refPosition.getX(), VisionConstants.maxBackCameraDepthDistance, 0.03);

                // Calculate the x speed for the robot to align with the target
                return -backCameraXPIDController.calculate(refPosition.getX(), VisionConstants.maxBackCameraDepthDistance);
            }

        } else { // If no lidars are valid, then use the camera distance to calculate the x speed
            isCollecting = false;

            // Check if the robot x position is in tolerance for the target x position
            xInTolerance = MathUtil.isNear(refPosition.getX(), VisionConstants.maxCameraDepthDistance, 0.03);

            // Calculate the x speed for the robot to align with the target
            return -cameraXPIDController.calculate(refPosition.getX(), VisionConstants.maxCameraDepthDistance);
        }
    }

    private double calculateTurnSpeed(double diffLidarDist, Pose3d refPosition) {
        // If both lidars are valid, then use the lidar distance to calculate the turn
        // speed
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

    public double[] getDistanceToSources() {
        if (!Constants.isBlueAlliance) {
            distanceToLeftSourceSide = swerve.getPose().getTranslation().getDistance(VisionConstants.kTagLayout.getTagPose(1).get().getTranslation().toTranslation2d());
            distanceToRightSourceSide = swerve.getPose().getTranslation().getDistance(VisionConstants.kTagLayout.getTagPose(2).get().getTranslation().toTranslation2d());
        } else {
            distanceToLeftSourceSide = swerve.getPose().getTranslation().getDistance(VisionConstants.kTagLayout.getTagPose(13).get().getTranslation().toTranslation2d());
            distanceToRightSourceSide = swerve.getPose().getTranslation().getDistance(VisionConstants.kTagLayout.getTagPose(12).get().getTranslation().toTranslation2d());
        }

        // Return the distance to the left and right sources
        return new double[] { distanceToLeftSourceSide, distanceToRightSourceSide };
    }

    public AlignState getAlignSourceSide() {
        // Get the distance to the left and right sources
        double[] distanceToSources = getDistanceToSources();

        // If the distance to the left source is less than the distance to the right
        // source, then return the left source
        if (distanceToSources[0] < distanceToSources[1]) {
            return AlignState.SourceLeft;
        } else { // If the distance to the right source is less than the distance to the left
                 // source, then return the right source
            return AlignState.SourceRight;
        }
    }

    private boolean isValidAlignTag(int tagID) {
        return VisionConstants.validAlignTags.contains(tagID);
    }

    public double getRightLidarDistance() {
        Logger.recordOutput("Vision/FrontRightLidarDistance", rightRange.getDistance().getValueAsDouble());
        return rightRange.getDistance().getValueAsDouble();
    }

    public double getLeftLidarDistance() {
        Logger.recordOutput("Vision/FrontLeftLidarDistance", leftRange.getDistance().getValueAsDouble());
        return leftRange.getDistance().getValueAsDouble();
    }

    public double getBackLidarDistance() {
        Logger.recordOutput("Vision/BackLidarDistance", backRange.getDistance().getValueAsDouble());
        return backRange.getDistance().getValueAsDouble();
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

    public boolean getBackLidarDetect() {
        return backRange.getIsDetected().getValue();
    }

    public static void setReefOrientation(ReefTargetOrientation orientation) {
        selectedReefOrientation = orientation;
    }

    public static void setPoleSide(ReefTargetSide side) {
        selectedPoleSide = side;
    }

    public static void setPoleLevel(ElevatorStates level) {
        selectedLevel = level;
    }

    public boolean isAligned() {
        SmartDashboard.putBoolean("X Alignment Happy", xInTolerance);
        SmartDashboard.putBoolean("Y Alignment Happy", yInTolerance);
        SmartDashboard.putBoolean("Z Alignment Happy", rotInTolerance);

        return xInTolerance && yInTolerance && rotInTolerance;
    }
}
