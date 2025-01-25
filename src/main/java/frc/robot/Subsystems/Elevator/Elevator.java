package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.ReefTargetLevel;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private ReefTargetLevel targetLevel;

  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Elevator instance not set");
    }
    return instance;
  }

  public void setReefTargetLevel(ReefTargetLevel level) {
    targetLevel = level;
    Logger.recordOutput("Elevator Target Level", targetLevel);
  }

  public ReefTargetLevel getReefTargetLevel() {
    return targetLevel;
  }

  public static Elevator setInstance(ElevatorIO io) {
    instance = new Elevator(io);
    return instance;
  }

  private Elevator(ElevatorIO io) {
    this.io = io;
    this.io.configurePID(0.2, 0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(ElevatorStates state) {
    io.setPosition(state.leftPosition, state.rightPosition);

    Logger.recordOutput("Elevator/State", state);
  }

  public boolean isAtState(ElevatorStates state, double tolerance) {
    return MathUtil.isNear(this.inputs.leftPositionRotations, state.leftPosition, tolerance)
        || MathUtil.isNear(this.inputs.rightPositionRotations, state.rightPosition, tolerance);
  }

  public void updateSim() {
    io.updateSim();
  }
}
