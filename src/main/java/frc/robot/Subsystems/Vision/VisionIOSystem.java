package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;
import org.photonvision.EstimatedRobotPose;

public class VisionIOSystem implements VisionIO {
  private final Swerve swerve;
  public static VisionIOSystem INSTANCE;

  public Matrix<N3, N1> defatultStdDev = VecBuilder.fill(0.5, 0.5, 99999);

  public final VisionCamera frontLeftCamera =
      new VisionCamera("CamOne", VisionConstants.camOneTransform);
  // public final VisionCamera frontRightCamera = new VisionCamera("Triceratops",
  // VisionConstants.triceratopsTransform);
  // public StdDevs stdDevsCalculation;
  private final Notifier allNotifier =
      new Notifier(
          () -> {
            frontLeftCamera.run();
            // frontRightCamera.run();
          });

  public VisionIOSystem() {
    this.swerve = Swerve.getInstance();

    allNotifier.setName("runAll");
    allNotifier.startPeriodic(0.02);
    // stdDevsCalculation = new StdDevs();
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

  }

  public void updatePose() {
    EstimatedRobotPose frontLeftPose = frontLeftCamera.grabLatestEstimatedPose();
    // EstimatedRobotPose frontRightPose =
    // frontRightCamera.grabLatestEstimatedPose();

    Matrix<N3, N1> frontLeftStdDevs = frontLeftCamera.grabLatestStdDev();
    // Matrix<N3, N1> frontRightStdDevs = frontRightCamera.grabLatestStdDev();

    addVisionReading(frontLeftPose, frontLeftStdDevs);
    // addVisionReading(frontRightPose, frontRightStdDevs);
  }

  public VisionCamera getFrontLeftCamera() {
    return frontLeftCamera;
  }

  @Override
  public void addVisionReading(EstimatedRobotPose pose, Matrix<N3, N1> stdDevs) {
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
