package frc.robot.Subsystems.Vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlignVision extends SubsystemBase {
  public AlignVisionIO io;
  public AlignVisionIOInputsAutoLogged inputs = new AlignVisionIOInputsAutoLogged();

  private static AlignVision INSTANCE;

  public static AlignVision getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new AlignVision(new AlignVisionIOSystem());
    }
    return INSTANCE;
  }

  public AlignVision(AlignVisionIO io) {
    this.io = io;
  }
}
