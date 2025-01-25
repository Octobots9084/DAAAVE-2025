package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;

public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public boolean frontLeftConnected = false;
    public boolean frontRightConnected = false;
    public boolean middleLeftConnected = false;
    public boolean middleRightConnected = false;
    public boolean backMiddleConnected = false;

    public PhotonPipelineResult frontLeftResult = null;
    public PhotonPipelineResult frontRightResult = null;
    public PhotonPipelineResult middleLeftResult = null;
    public PhotonPipelineResult middleRightResult = null;
    public PhotonPipelineResult backMiddleResult = null;
  }

  public default void updateInputs(VisionIOInputs inputs) {}

  public void updatePose();

  public void closeNotifiers();

  public void addVisionReading(EstimatedRobotPose pose, Matrix<N3, N1> visionMeasurementStdDevs);
}
