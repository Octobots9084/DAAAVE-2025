package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static Wrist instance;
  private WristStates state = WristStates.FOURTYFIVE;

  public static Wrist getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Wrist instance not set");
    }
    return instance;
  }

  public static Wrist setInstance(WristIO io) {
    instance = new Wrist(io);
    return instance;
  }

  public Wrist(WristIO io) {
    this.io = io;
    io.configurePID(8, 0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public void setState(WristStates state) {
    io.setPosition(state.wristPosition);
    Logger.recordOutput("Wrist/State", state);
    this.state = state;
  }

  public WristStates getState() {
    return this.state;
  }

  public boolean isAtState(WristStates state, double tolerance) {
    return MathUtil.isNear(this.inputs.wristPositionRotations, state.wristPosition, tolerance);
  }

  public void updateSim() {
    io.updateSim();
  }
}
