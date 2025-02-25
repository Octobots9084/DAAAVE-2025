package frc.robot.Subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
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
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.estimation.TargetModel;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.littletonrobotics.junction.Logger;

public class VisionIOSim implements VisionIO {
    private final Swerve swerve;
    public static VisionIOSystem INSTANCE;

  VisionSystemSim visionSim = new VisionSystemSim("main");
  TargetModel targetModel = TargetModel.kAprilTag36h11;
  SimCameraProperties cameraProp = new SimCameraProperties();
//   SimCameraProperties cameraProp2 = new SimCameraProperties();
//   SimCameraProperties cameraProp3 = new SimCameraProperties();
//   SimCameraProperties cameraProp4 = new SimCameraProperties();
  AprilTagFieldLayout tagLayout;

  public Matrix<N3, N1> defatultStdDev = VecBuilder.fill(0, 0, 99999);

  public final VisionCamera frontLeftCamera = new VisionCamera("Rex", VisionConstants.transformFrontLeftToRobot);
//   public final VisionCamera frontRightCamera = new VisionCamera("Tyrannosaurus", VisionConstants.transformFrontRightToRobot);
//   public final VisionCamera middleRightCamera = new VisionCamera("Oviraptor", VisionConstants.transformMiddleRightToRobot);
//   public final VisionCamera middleLeftCamera = new VisionCamera("Brontosaurus", VisionConstants.transformMiddleLeftToRobot);

  public StdDevs stdDevsCalculation;
//   public StdDevs stdDevsCalculation2;
//   public StdDevs stdDevsCalculation3;
//   public StdDevs stdDevsCalculation4;
  
  // The simulation of this camera. Its values used in real robot code will be
  // updated.
  PhotonCameraSim cameraSim = new PhotonCameraSim(frontLeftCamera.getCamera(), cameraProp);
//   PhotonCameraSim cameraSim2 = new PhotonCameraSim(frontRightCamera.getCamera(), cameraProp2);
//   PhotonCameraSim cameraSim3 = new PhotonCameraSim(middleRightCamera.getCamera(), cameraProp3);
//   PhotonCameraSim cameraSim4 = new PhotonCameraSim(middleLeftCamera.getCamera(), cameraProp4);

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
    cameraProp.setCalibration(1280, 800, Rotation2d.fromDegrees(77.63));
    // cameraProp2.setCalibration(1280, 800, Rotation2d.fromDegrees(77.63));
    // Approximate detection noise with average and standard deviation error in
    // pixels.
    cameraProp.setCalibError(0., 0.);
    // cameraProp2.setCalibError(0.33, 0.33);
    // Set the camera image capture framerate (Note: this is limited by robot loop
    // rate).
    cameraProp.setFPS(20);
    // cameraProp2.setFPS(20);
    // The average and standard deviation in milliseconds of image data latency.
    cameraProp.setAvgLatencyMs(35);
    // cameraProp2.setAvgLatencyMs(35);
    cameraProp.setLatencyStdDevMs(5);
    // cameraProp2.setLatencyStdDevMs(5);

    Translation3d robotToCameraTrl = new Translation3d(VisionConstants.transformFrontLeftToRobot.getX(), VisionConstants.transformFrontLeftToRobot.getY(), VisionConstants.transformFrontLeftToRobot.getZ());
    // Translation3d robotToCameraTrl2 = new Translation3d(VisionConstants.transformFrontRightToRobot.getX(), VisionConstants.transformFrontRightToRobot.getY(), VisionConstants.transformFrontRightToRobot.getZ());
    Rotation3d robotToCameraRot = VisionConstants.transformFrontLeftToRobot.getRotation();
    // Rotation3d robotToCameraRot2 = VisionConstants.transformFrontLeftToRobot.getRotation();
    Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);
    // Transform3d robotToCamera2 = new Transform3d(robotToCameraTrl2, robotToCameraRot2);

    // Add this camera to the vision system simulation with the given
    // robot-to-camera transform.
    visionSim.addCamera(cameraSim, robotToCamera);
    // visionSim.addCamera(cameraSim2, robotToCamera2);

    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
    stdDevsCalculation = new StdDevs();
    // SmartDashboard.putData(visionSim.getDebugField());
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
    Optional<Pose2d> simPose = swerve.getSimPose();
    if (simPose.isPresent()) {
        visionSim.update(simPose.get());

    }
  }

  public void updatePose() {
    EstimatedRobotPose frontLeftPose = frontLeftCamera.grabLatestEstimatedPose();
    // EstimatedRobotPose frontRightPose = frontRightCamera.grabLatestEstimatedPose();

        Matrix<N3, N1> frontLeftStdDevs = frontLeftCamera.grabLatestStdDev();
        // Matrix<N3, N1> frontRightStdDevs = frontRightCamera.grabLatestStdDev();

        addVisionReading("Front Left", frontLeftPose, frontLeftStdDevs);
        // addVisionReading(frontRightPose, frontRightStdDevs);
    }

    public VisionCamera getFrontLeftCamera() {
        return frontLeftCamera;
    }

  @Override
  public void addVisionReading(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
    if (pose != null && stdDevs != null) {
      Pose2d pose2d = pose.estimatedPose.toPose2d();
      Logger.recordOutput("FrontLeftPositionEstimate", pose2d);
      stdDevsCalculation.update(pose2d.getX(), pose2d.getY(),
      pose2d.getRotation().getRadians());
      SmartDashboard.putNumber("StdDevsX",
      stdDevsCalculation.getStandardDeviationX());
      SmartDashboard.putNumber("StdDevsY",
      stdDevsCalculation.getStandardDeviationY());

      SmartDashboard.putNumber("StdDevsRot",
      stdDevsCalculation.getStandardDeviationRotation());
      swerve.addVisionReading(pose2d, pose.timestampSeconds, stdDevs);
    }
  }
}
