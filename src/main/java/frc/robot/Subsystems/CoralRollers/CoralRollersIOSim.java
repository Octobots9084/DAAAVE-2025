package frc.robot.Subsystems.CoralRollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class CoralRollersIOSim implements CoralRollersIO {
  CoralRollersIOSystems sparkMaxes = new CoralRollersIOSystems();
  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1), DCMotor.getNeo550(1));
  AnalogInputSim rearBeamSim = new AnalogInputSim(2);
  AnalogInputSim mouthBeamSim = new AnalogInputSim(3);

  @Override
  public void updateInputs(CoralRollersIOInputs inputs) {
    inputs.velocityRPM = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.hasCoral = this.hasCoral();
  }

  @Override
  public void setVoltage(double volts) {
    motorSim.setInput(MathUtil.clamp(volts, -12.0, 12.0));
  }

  @Override
  public void updateSim() {
    motorSim.update(0.02);
  }

  // TODO - Actually change these values
  @Override
  public boolean hasCoral() {
    return this.rearBeamSim.getVoltage() != 0;
  }
}
