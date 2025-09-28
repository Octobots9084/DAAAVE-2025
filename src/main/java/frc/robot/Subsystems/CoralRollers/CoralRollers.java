package frc.robot.Subsystems.CoralRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Commands.ReefSelection.manager;
import frc.robot.Subsystems.Elevator.ElevatorStates;

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
    if ((manager.level == ElevatorStates.LEVEL3 || manager.level == ElevatorStates.LEVEL2) && state ==CoralRollersState.OUTPUT)
    {
        io.setVoltage(state.voltage-2.0);
    }
    else
    {
        io.setVoltage(state.voltage);
    }
    
  }

  public CoralRollersState getState() {
    return coralRollersState;
  }

  public boolean HasCoral() {
    return io.HasCoral();
  }

  public boolean IsIntaking() {
    return io.IsIntaking();
  }

  public boolean clawFrontSensorTriggered() {
    return io.clawFrontSensorTriggered();
  }

  public boolean clawBackSensorTriggered() {
    return io.clawBackSensorTriggered();
  }

  public void rotateBy(double movement) {
    io.rotateBy(movement);
  }

  public void updateSim() {
    io.updateSim();
  }

  public boolean isStalled() {
    return io.isStalled();
  }

}
