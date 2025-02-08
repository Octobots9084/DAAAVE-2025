package frc.robot.Subsystems.CoralRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralRollers extends SubsystemBase {
  public final CoralRollersIO io;
  private static CoralRollers instance = null;
  private final CoralRollersIOInputsAutoLogged inputs = new CoralRollersIOInputsAutoLogged();
  private CoralRollersState coralRollersState = CoralRollersState.STOPPED;

  public static CoralRollers getInstance() {
    if (instance == null) {
      throw new IllegalStateException("CoralRollers instance not set");
    }
    return instance;
  }

  public static CoralRollers setInstance(CoralRollersIO io) {
    instance = new CoralRollers(io);
    return instance;
  }

  public CoralRollers(CoralRollersIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralRollers", inputs);
  }

  public void setState(CoralRollersState state) {
    coralRollersState = state;
    io.setVoltage(state.voltage);
  }

  public CoralRollersState getState() {
    return coralRollersState;
  }

  public boolean hasCoral() {
    return io.hasCoral();
  }

  public void updateSim() {
    io.updateSim();
  }
}
