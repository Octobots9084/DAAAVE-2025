package frc.robot.Subsystems.CoralRollers;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.simulation.AnalogInputSim;

public class CoralRollersIOSim implements CoralRollersIO {
  CoralRollersIOSystems sparkMaxes = new CoralRollersIOSystems();
  SparkRelativeEncoderSim SparkSim = new SparkRelativeEncoderSim(sparkMaxes.getMotor());
  AnalogInputSim rearBeamSim = new AnalogInputSim(2);
  AnalogInputSim mouthBeamSim = new AnalogInputSim(3);
  private double appliedVolts = 11;

  @Override
  public void updateInputs(CoralRollersIOInputs inputs) {
    inputs.velocityRPM = SparkSim.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = 0;
    inputs.hasCoral = this.hasCoral();
    // inputs.temperature = 0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }

  // TODO - Actually change these values
  @Override
  public boolean hasCoral() {
    return this.rearBeamSim.getVoltage() != 0;
  }

  @Override
  public boolean isIntaking() {
    return this.mouthBeamSim.getVoltage() != 0;
  }
}
