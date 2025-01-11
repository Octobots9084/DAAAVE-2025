package frc.robot.Subsystems.CoralRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralRollers extends SubsystemBase {
  private final CoralRollersIOSystems io;
  private static CoralRollers instance = null;
  private final CoralRollersIOInputsAutoLogged inputs = new CoralRollersIOInputsAutoLogged();

  public static CoralRollers getInstance() {
    if (instance == null) {
      instance = new CoralRollers(new CoralRollersIOSystems());
    }
    return instance;
  }

  public CoralRollers(CoralRollersIOSystems io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralRollers", inputs);
  }

  public void setState(CoralRollersState state) {
    io.setVoltage(state.voltage);
  }
}
