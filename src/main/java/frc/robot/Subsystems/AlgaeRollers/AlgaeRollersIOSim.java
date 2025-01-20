package frc.robot.Subsystems.AlgaeRollers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
  AnalogInputSim beamInputSim = new AnalogInputSim(0);
  DCMotorSim motorSim =
      new DCMotorSim(
          LinearSystemId.createDCMotorSystem(DCMotor.getNeo550(1), 0.025, 1), DCMotor.getNeo550(1));

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.velocityRPM = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = motorSim.getInputVoltage();
    inputs.currentAmps = motorSim.getCurrentDrawAmps();
    inputs.beamValue = this.hasAlgae();
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
  public boolean hasAlgae() {
    return this.beamInputSim.getVoltage() != 0;
  }
}
