package frc.robot.Subsystems.AlgaeRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollers extends SubsystemBase {
  private final AlgaeRollersIO io;
  private final AlgaeRollersIOInputsAutoLogged inputs = new AlgaeRollersIOInputsAutoLogged();
  private static AlgaeRollers instance = null;

  public static AlgaeRollers getInstance() {
    if (instance == null) {
      throw new IllegalStateException("AlgaeRollers instance not set");
    }
    return instance;
  }

  public static AlgaeRollers setInstance(AlgaeRollersIO io) {
    instance = new AlgaeRollers(io);
    return instance;
  }

  public AlgaeRollers(AlgaeRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeRollers", inputs);
  }

  public void setState(AlgaeRollersStates state) {
    io.setVoltage(state.voltage);
    Logger.recordOutput("AlgaeRollers/State", state);
  }

  public void updateSim() {
    io.updateSim();
  }
}
