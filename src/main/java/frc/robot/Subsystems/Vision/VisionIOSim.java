package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import java.io.IOException;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
    private final Swerve swerve;
    public static VisionIOSystem INSTANCE;

    VisionSystemSim visionSim = new VisionSystemSim("main");
    TargetModel targetModel = TargetModel.kAprilTag36h11;
    SimCameraProperties cameraProp = new SimCameraProperties();
    AprilTagFieldLayout tagLayout;

    public Matrix<N3, N1> defatultStdDev = VecBuilder.fill(0.5, 0.5, 99999);

    public final VisionCamera frontLeftCamera = new VisionCamera("Rex", VisionConstants.transformFrontLeftToRobot);
    public final VisionCamera frontRightCamera = new VisionCamera("Tyrannosaurus",
            VisionConstants.transformFrontRightToRobot);
    public final VisionCamera middleRightCamera = new VisionCamera("Oviraptor",
            VisionConstants.transformMiddleRightToRobot);
    public final VisionCamera middleLeftCamera = new VisionCamera("Brontosaurus",
            VisionConstants.transformMiddleLeftToRobot);

    // The simulation of this camera. Its values used in real robot code will be
    // updated.
    PhotonCameraSim cameraSim = new PhotonCameraSim(frontLeftCamera.getCamera(), cameraProp);

    private final Notifier allNotifier = new Notifier(
            () -> {
                frontLeftCamera.run();
                frontRightCamera.run();
                middleRightCamera.run();
                middleLeftCamera.run();
            });

    public VisionIOSim() {
        try {
            tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }
        this.swerve = Swerve.getInstance();

        visionSim.addAprilTags(tagLayout);
        // A 640 x 480 camera with a 100 degree diagonal FOV.
        cameraProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
        // Approximate detection noise with average and standard deviation error in
        // pixels.
        cameraProp.setCalibError(0.25, 0.08);
        // Set the camera image capture framerate (Note: this is limited by robot loop
        // rate).
        cameraProp.setFPS(20);
        // The average and standard deviation in milliseconds of image data latency.
        cameraProp.setAvgLatencyMs(35);
        cameraProp.setLatencyStdDevMs(5);

        // TODO - fix these positions
        // example camera is mounted 0.1 meters forward and 0.5 meters up from the robot
        // pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z =
        // 0)
        Translation3d robotToCameraTrl = new Translation3d(0.1, 0, 0.5);
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        // Add this camera to the vision system simulation with the given
        // robot-to-camera transform.
        visionSim.addCamera(cameraSim, robotToCamera);

        allNotifier.setName("runAll");
        allNotifier.startPeriodic(0.02);
        // stdDevsCalculation = new StdDevs();
        SmartDashboard.putData(visionSim.getDebugField());
    }

    public void closeNotifiers() {
        allNotifier.close();
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.frontLeftConnected = frontLeftCamera.isConnected();
        // inputs.frontRightConnected = frontRightCamera.isConnected();

        inputs.frontLeftResult = frontLeftCamera.grabLatestResult();
        // inputs.frontRightResult = frontRightCamera.grabLatestResult();
        visionSim.update(swerve.getPose());
    }

    public void updatePose() {
        EstimatedRobotPose frontLeftPose = frontLeftCamera.grabLatestEstimatedPose();
        // EstimatedRobotPose frontRightPose =
        // frontRightCamera.grabLatestEstimatedPose();

        Matrix<N3, N1> frontLeftStdDevs = frontLeftCamera.grabLatestStdDev();
        // Matrix<N3, N1> frontRightStdDevs = frontRightCamera.grabLatestStdDev();

        addVisionReading("Front Left", frontLeftPose, frontLeftStdDevs);
        // addVisionReading(frontRightPose, frontRightStdDevs);
    }

    public VisionCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

    @Override
    public void addVisionReading(String cameraName, EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
        if (pose != null && stdDevs != null) {
            Pose2d pose2d = pose.estimatedPose.toPose2d();
            // stdDevsCalculation.update(pose2d.getX(), pose2d.getY(),
            // pose2d.getRotation().getRadians());
            // SmartDashboard.putNumber("StdDevsX",
            // stdDevsCalculation.getStandardDeviationX());
            // SmartDashboard.putNumber("StdDevsY",
            // stdDevsCalculation.getStandardDeviationY());

            // SmartDashboard.putNumber("StdDevsRot",
            // stdDevsCalculation.getStandardDeviationRotation());
            swerve.addVisionReading(pose2d, pose.timestampSeconds, stdDevs);
        }
    }
}
