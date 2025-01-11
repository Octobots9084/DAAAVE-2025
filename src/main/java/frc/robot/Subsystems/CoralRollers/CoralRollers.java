package frc.robot.Subsystems.CoralRollers;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class CoralRollers extends SubsystemBase {
  private final CoralRollersIOSystems io;
  private static CoralRollers instance = null;
  private final CoralRollersIOInputsAutoLogged inputs = new CoralRollersIOInputsAutoLogged();

  public static CoralRollers getInstance() {
    if (instance == null) {
      instance = new CoralRollers();
    }
    return instance;
  }

  public CoralRollers() {
    this.io = new CoralRollersIOSystems();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("CoralRollers", inputs);
  }

  public void setState(CoralRollersState state) {
    io.setVoltage(state.voltage);
  }

  public boolean isIntaking() {
    return this.io.isIntaking();
  }

  public boolean hasCoral() {
    return this.io.hasCoral();
  }
}
