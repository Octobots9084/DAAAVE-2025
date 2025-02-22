package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import org.photonvision.EstimatedRobotPose;

public class VisionIOSystem implements VisionIO {
  private final Swerve swerve;
  public static VisionIOSystem INSTANCE;

  public Matrix<N3, N1> defatultStdDev = VecBuilder.fill(0.5, 0.5, 99999);

  public final VisionCamera frontLeftCamera = new VisionCamera("Rex", VisionConstants.transformFrontLeftToRobot);
  public final VisionCamera frontRightCamera = new VisionCamera("Tyrannosaurus",
      VisionConstants.transformFrontRightToRobot);
  public final VisionCamera middleRightCamera = new VisionCamera("Oviraptor",
      VisionConstants.transformMiddleRightToRobot);
  public final VisionCamera middleLeftCamera = new VisionCamera("Brontosaurus",
      VisionConstants.transformMiddleLeftToRobot);

  public StdDevs stdDevsCalculation;
  private final Notifier allNotifier = new Notifier(
      () -> {
        frontLeftCamera.run();
        frontRightCamera.run();
        middleLeftCamera.run();
        middleRightCamera.run();

      });

  public VisionIOSystem() {
    this.swerve = Swerve.getInstance();

    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
    stdDevsCalculation = new StdDevs();
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

    inputs.frontLeftResult = frontLeftCamera.grabLatestResult();
    inputs.frontRightResult = frontRightCamera.grabLatestResult();
    inputs.middleLeftResult = middleLeftCamera.grabLatestResult();
    inputs.middleRightResult = middleRightCamera.grabLatestResult();
  }

  public void updatePose() {
    EstimatedRobotPose frontLeftPose = frontLeftCamera.grabLatestEstimatedPose();
    EstimatedRobotPose frontRightPose = frontRightCamera.grabLatestEstimatedPose();
    EstimatedRobotPose middleLeftPose = middleLeftCamera.grabLatestEstimatedPose();
    EstimatedRobotPose middleRightPose = middleRightCamera.grabLatestEstimatedPose();

    Matrix<N3, N1> frontLeftStdDevs = frontLeftCamera.grabLatestStdDev();
    Matrix<N3, N1> frontRightStdDevs = frontRightCamera.grabLatestStdDev();
    Matrix<N3, N1> middleLeftStdDevs = middleLeftCamera.grabLatestStdDev();
    Matrix<N3, N1> middleRightStdDevs = middleRightCamera.grabLatestStdDev();

    addVisionReading(frontLeftPose, frontLeftStdDevs);
    addVisionReading(frontRightPose, frontRightStdDevs);
    addVisionReading(middleLeftPose, middleLeftStdDevs);
    addVisionReading(middleRightPose, middleRightStdDevs);
  }

  public VisionCamera getFrontLeftCamera() {
    return frontLeftCamera;
  }

  @Override
  public void addVisionReading(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
    if (pose != null && stdDevs != null) {
      Pose2d pose2d = pose.estimatedPose.toPose2d();
      stdDevsCalculation.update(pose2d.getX(), pose2d.getY(),
          pose2d.getRotation().getRadians());
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
