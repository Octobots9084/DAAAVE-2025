package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
  private final WristIOSparkMax io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private static Wrist instance;

  public static void setInstance(Wrist subsystem) {
    Wrist.instance = subsystem;
  }

  public static Wrist getInstance() {
    return instance;
  }

  /** Creates a new Flywheel. */
  public Wrist(WristIOSparkMax io) {
    this.io = io;
    io.configurePID(8, 0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  /** Run closed loop at the specified velocity. */
  public void setState(WristStates state) {
    io.setPosition(state.wristPosition);
    Logger.recordOutput("Wrist/State", state);
  }

  public boolean isAtState(WristStates state, double tolerance) {
    return MathUtil.isNear(this.inputs.wristPositionRotations, state.wristPosition, tolerance);
  }

  public void setPosition(double target) {
    io.setPosition(target);
  }
}
