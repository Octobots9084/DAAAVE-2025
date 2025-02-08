package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.States.AlignState;
import frc.robot.States.ReefTargetLevel;
import frc.robot.States.ReefTargetOrientation;
import frc.robot.States.ReefTargetSide;
import org.littletonrobotics.junction.AutoLog;
import org.photonvision.targeting.PhotonPipelineResult;

public interface AlignVisionIO {

  @AutoLog
  public static class AlignVisionIOInputs {
  }

  public ChassisSpeeds getAlignChassisSpeeds(AlignState state);

  public double getRightLidarDistance();

  public double getLeftLidarDistance();

  public boolean areBothLidarsValid();

  public boolean getRightLidarDetect();

  public boolean getLeftLidarDetect();

  public static void setReefOrientation(ReefTargetOrientation orientation) {
  }

  public static void setPoleSide(ReefTargetSide side) {
  }

  public static void setPoleLevel(ReefTargetLevel level) {
  }
}
