package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private ElevatorStates targetLevel = ElevatorStates.LOW;

  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Elevator instance not set");
    }
    return instance;
  }

  public void setReefTargetLevel(ElevatorStates level) {
    targetLevel = level;
    Logger.recordOutput("Elevator Target Level", targetLevel);
  }

  public ElevatorStates getReefTargetLevel() {
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
    io.setPosition(state.position, state.position);

    Logger.recordOutput("Elevator/State", state);
  }

  public boolean isAtState(ElevatorStates state, double tolerance) {
    return MathUtil.isNear(this.inputs.leftPositionRotations, targetLevel.position, tolerance)
        || MathUtil.isNear(
            this.inputs.rightPositionRotations, targetLevel.position, tolerance);
  }

  public boolean isAtState(double tolerance) {
    return MathUtil.isNear(this.inputs.leftPositionRotations, targetLevel.position, tolerance)
        || MathUtil.isNear(
            this.inputs.rightPositionRotations, targetLevel.position, tolerance);
  }

  public void updateSim() {
    io.updateSim();
  }
}
