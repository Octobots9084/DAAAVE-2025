package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

public class VisionIOSystem implements VisionIO {
    private final Swerve swerve;
    public static VisionIOSystem INSTANCE;

    public final PieceVisionCamera algaeAlignCamera = new PieceVisionCamera("HD_Web_Camera", VisionConstants.tranformCenterRobotToAlgaeCamera); //TODO change name to actual photon vision name to be velociraptor and update the transform 3d to the accurate one on robot.
    public final VisionCamera frontLeftCamera = new VisionCamera("FrontLeft", VisionConstants.transformFrontLeftToRobot);
    public final VisionCamera frontRightCamera = new VisionCamera("FrontRight",
            VisionConstants.transformFrontRightToRobot);
    public final VisionCamera middleRightCamera = new VisionCamera("SideRight",
            VisionConstants.transformMiddleRightToRobot);
    public final VisionCamera middleLeftCamera = new VisionCamera("SideLeft",
            VisionConstants.transformMiddleLeftToRobot);
    public final VisionCamera backCamera = new VisionCamera("Back", VisionConstants.transformBackToRobot);

    private final Notifier allNotifier = new Notifier(
            () -> {
                frontLeftCamera.run();
                frontRightCamera.run();
                middleLeftCamera.run();
                middleRightCamera.run();
                backCamera.run();
            });

    public VisionIOSystem() {
        this.swerve = Swerve.getInstance();

        allNotifier.setName("runAll");
        allNotifier.startPeriodic(0.02);
    }

    public void closeNotifiers() {
        allNotifier.close();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.frontLeftConnected = frontLeftCamera.isConnected();
        inputs.frontRightConnected = frontRightCamera.isConnected();
        inputs.middleLeftConnected = middleLeftCamera.isConnected();
        inputs.middleRightConnected = middleRightCamera.isConnected();
        inputs.backMiddleConnected = backCamera.isConnected();

        inputs.frontLeftResult = frontLeftCamera.grabLatestResult();
        inputs.frontRightResult = frontRightCamera.grabLatestResult();
        inputs.middleLeftResult = middleLeftCamera.grabLatestResult();
        inputs.middleRightResult = middleRightCamera.grabLatestResult();
        inputs.backMiddleResult = backCamera.grabLatestResult();

        inputs.leftLidarDistance = AlignVision.getInstance().getLeftLidarDistance();
        inputs.rightLidarDistance = AlignVision.getInstance().getRightLidarDistance();

        inputs.algaeCameraHasTargets = algaeAlignCamera.hasTargets();
        inputs.algaeCenterOffset = algaeAlignCamera.getCenterOffset();
    }

    public void updatePose() {
        EstimatedRobotPose frontLeftPose = frontLeftCamera.grabLatestEstimatedPose();
        EstimatedRobotPose frontRightPose = frontRightCamera.grabLatestEstimatedPose();
        EstimatedRobotPose middleLeftPose = middleLeftCamera.grabLatestEstimatedPose();
        EstimatedRobotPose middleRightPose = middleRightCamera.grabLatestEstimatedPose();
        EstimatedRobotPose backMiddlePose = backCamera.grabLatestEstimatedPose();

        Matrix<N3, N1> frontLeftStdDevs = frontLeftCamera.grabLatestStdDev();
        Matrix<N3, N1> frontRightStdDevs = frontRightCamera.grabLatestStdDev();
        Matrix<N3, N1> middleLeftStdDevs = middleLeftCamera.grabLatestStdDev();
        Matrix<N3, N1> middleRightStdDevs = middleRightCamera.grabLatestStdDev();
        Matrix<N3, N1> backStdDevs = backCamera.grabLatestStdDev();

        addVisionReading("Front Left", frontLeftPose, frontLeftStdDevs);
        addVisionReading("Front Right", frontRightPose, frontRightStdDevs);
        addVisionReading("Middle Left", middleLeftPose, middleLeftStdDevs);
        addVisionReading("Middle Right", middleRightPose, middleRightStdDevs);
        addVisionReading("Back", backMiddlePose, backStdDevs);
    }

    public VisionCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

    public PieceVisionCamera getAlgaeCamera(){
        return algaeAlignCamera;
    }

    @Override
    public void addVisionReading(String cameraName, EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
        if (pose != null && stdDevs != null) {
            Pose2d pose2d = pose.estimatedPose.toPose2d();
            swerve.addVisionReading(pose2d, pose.timestampSeconds, stdDevs);
        }
    }
}
