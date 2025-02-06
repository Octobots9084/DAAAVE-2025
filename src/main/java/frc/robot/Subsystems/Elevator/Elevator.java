package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private ElevatorStates targetLevel = ElevatorStates.LOW;

  //TODO add actual input chanel
  public DigitalInput toplimitSwitch = new DigitalInput(0);
  public DigitalInput bottomlimitSwitch = new DigitalInput(1);

  private static Elevator instance;

  public static Elevator getInstance() {
    if (instance == null) {
      throw new IllegalStateException("Elevator instance not set");
    }
    return instance;
  }

  public ElevatorStates getReefTargetLevel() {
    return targetLevel;
  }

  public double getPosition() {
    return this.io.getPosition();
  }

  public static Elevator setInstance(ElevatorIO io) {
    instance = new Elevator(io);
    return instance;
  }

  private Elevator(ElevatorIO io) {
    this.io = io;
    // this.io.configurePID(0.7, 0, 0);
  }

  @Override
  public void periodic() {
    if (bottomlimitSwitch.get()) {
      // We are going up and top limit is tripped so stop
      if (io.getPosition() < 0){
        io.setPosition(0 ,0);
        io.getLeftMotor().getAlternateEncoder().setPosition(0);
      }
        
    }
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void setState(ElevatorStates state) {
    io.setPosition(state.position, state.position);
    targetLevel = state;
    Logger.recordOutput("Elevator/State", state);
  }

  public ElevatorStates getState() {
    return targetLevel;
  }

  public void manuelSetTargetPosistion(double position) {
    io.setPosition(position, position);
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
