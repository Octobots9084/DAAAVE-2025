package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIOSparkMax io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private static Elevator instance;

  public static void setInstance(Elevator subsystem) {
    Elevator.instance = subsystem;
  }

  public static Elevator getInstance() {
    return instance;
  }
  /** Creates a new Elevator. */
  public Elevator(ElevatorIOSparkMax io) {
    this.io = io;
    io.configurePID(8, 0, 0);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  /** Run closed loop at the specified velocity. */
  public void setState(ElevatorStates state) {
    io.setPosition(state.leftPosition, state.rightPosition);

    Logger.recordOutput("Elevator/State", state);
  }

  public boolean isAtState(ElevatorStates state, double tolerance) {
    return MathUtil.isNear(
            this.inputs.leftPositionRotations, state.leftPosition, tolerance)
        && MathUtil.isNear(
            this.inputs.rightPositionRotations, state.rightPosition, tolerance);
  }
}
