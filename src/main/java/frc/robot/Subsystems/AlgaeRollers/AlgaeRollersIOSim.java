package frc.robot.Subsystems.AlgaeRollers;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;

public class AlgaeRollersIOSim implements AlgaeRollersIO {
  AlgaeRollersIOSystems sparkMaxes = new AlgaeRollersIOSystems();
  SparkRelativeEncoderSim SparkSim = new SparkRelativeEncoderSim(sparkMaxes.getMotor());
  AnalogInputSim beamInputSim = new AnalogInputSim(0);
  private double appliedVolts = 11;

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    inputs.velocityRPM = SparkSim.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = 0;
    inputs.beamValue = this.hasAlgae();
    // inputs.temperature = 0;
  }

  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  // TODO - Actually change these values
  @Override
  public boolean hasAlgae() {
    return this.beamInputSim.getVoltage() != 0;
  }
}
