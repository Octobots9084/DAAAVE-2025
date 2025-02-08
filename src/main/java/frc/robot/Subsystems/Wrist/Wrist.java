package frc.robot.Subsystems.Wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems.Elevator.ElevatorStates;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();
  private WristStates targetState = WristStates.VERTICAL;
  private static Wrist instance;
  public final double MaxAngle = 0.668;
  public final double MinAngle = 0;
  public final double UnderCrossbarAngle = 0.261;

  public static Wrist getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Wrist instance not set");
    }
    return instance;
  }

  public void setOffset(double offset){
    io.setOffset(offset);
  }

  public SparkMax getWristMotor() {
    return io.getWristMotor();
  }

  public static Wrist setInstance(WristIO io) {
    instance = new Wrist(io);
    return instance;
  }

  public Wrist(WristIO io) {
    this.io = io;
  }
  
  public double getPosition()
  {
    return this.io.getPosition();
  }

  @Override
  public void periodic() {

    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
  }

  public void setState(WristStates state, ClosedLoopSlot slot) {
    io.setPosition(state.wristPosition, slot);
    targetState = state;
    Logger.recordOutput("Wrist/State", state);
    Logger.recordOutput("Wrist/CommandedPosition", state.wristPosition);
  }

  public WristStates getState() {
    return targetState;
  }

  public boolean isAtState(WristStates state, double tolerance) {
    return MathUtil.isNear(this.inputs.wristPositionRotations, state.wristPosition, tolerance);
  }

  public boolean isAtState(ElevatorStates state, double tolerance) {
    WristStates wriststate = WristStates.LOW;
    // TODO add the actual WristStates and elevatorStates
    if (state == ElevatorStates.LOW)
      wriststate = WristStates.LOW;
    else if (state == ElevatorStates.LEVEL1)
      wriststate = WristStates.VERTICAL;
    else if (state == ElevatorStates.LEVEL2)
      wriststate = WristStates.FOURTYFIVE;
    else if (state == ElevatorStates.LEVEL3)
      wriststate = WristStates.FOURTYFIVE;
    else if (state == ElevatorStates.LEVEL4)
      wriststate = WristStates.HORIZONTAL;
    else if (state == ElevatorStates.INTAKE)
      wriststate = WristStates.VERTICAL;

    return MathUtil.isNear(this.inputs.wristPositionRotations, wriststate.wristPosition, tolerance);
  }

  // overloads the setstate method to allow converting between wrist and elevator
  // state as they
  // should be corrilateds
  public void setState(ElevatorStates state, ClosedLoopSlot slot) {
    WristStates wriststate = WristStates.LOW;
    // TODO add the actual WristStates and elevatorStates
    if (state == ElevatorStates.LOW)
      wriststate = WristStates.LOW;
    else if (state == ElevatorStates.LEVEL1)
      wriststate = WristStates.VERTICAL;
    else if (state == ElevatorStates.LEVEL2)
      wriststate = WristStates.FOURTYFIVE;
    else if (state == ElevatorStates.LEVEL3)
      wriststate = WristStates.FOURTYFIVE;
    else if (state == ElevatorStates.LEVEL4)
      wriststate = WristStates.HORIZONTAL;
    else if (state == ElevatorStates.INTAKE)
      wriststate = WristStates.VERTICAL;

    targetState = wriststate;
    io.setPosition(wriststate.wristPosition, slot);
    Logger.recordOutput("Wrist/State", wriststate);
  }

  public boolean IsInsideRobot() {
    double wristPosition = this.getWristMotor().getAbsoluteEncoder().getPosition();

    return (wristPosition < this.UnderCrossbarAngle);
  }

  public void updateSim() {
    io.updateSim();
  }
}
