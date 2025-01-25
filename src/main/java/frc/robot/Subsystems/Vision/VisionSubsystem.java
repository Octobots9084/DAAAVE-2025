package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.littletonrobotics.junction.Logger;

public class VisionSubsystem extends SubsystemBase {
  VisionIO io;
  VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  public static VisionSubsystem INSTANCE;

  public static VisionSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new VisionSubsystem(new VisionIOSystem());
    }

    return INSTANCE;
  }

  public VisionSubsystem(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    io.updatePose();
    Logger.processInputs("Vision", inputs);
    if (VisionConstants.USE_VISION) {
      io.updatePose();

    } else {
      io.closeNotifiers();
    }
  }
}
