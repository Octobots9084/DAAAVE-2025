package frc.robot.Subsystems.AlgaeRollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class AlgaeRollers extends SubsystemBase {
  private final AlgaeRollersIOSystems io;
  private final AlgaeRollersIOInputsAutoLogged inputs = new AlgaeRollersIOInputsAutoLogged();
  private static AlgaeRollers instance = null;

  public static AlgaeRollers getInstance() {
    if (instance == null) {
      instance = new AlgaeRollers(new AlgaeRollersIOSystems());
    }
    return instance;
  }

  public AlgaeRollers(AlgaeRollersIOSystems io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("AlgaeRollers", inputs);
  }

  /** Run closed loop at the specified velocity. */
  public void setState(AlgaeRollersStates state) {
    io.setVoltage(state.voltage);
    Logger.recordOutput("AlgaeRollers/State", state);
  }

  public boolean isAtState(AlgaeRollersStates state, double tolerance) {
    return MathUtil.isNear(state.voltage, io.getVoltage(), tolerance);
  }
}
