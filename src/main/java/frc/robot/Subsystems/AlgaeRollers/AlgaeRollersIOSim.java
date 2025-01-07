package frc.robot.Subsystems.AlgaeRollers;

import edu.wpi.first.math.MathUtil;
import com.revrobotics.sim.SparkRelativeEncoderSim;

public class AlgaeRollersIOSim implements AlgaeRollersIO {

  private double appliedVolts = 11;

  @Override
  public void updateInputs(AlgaeRollersIOInputs inputs) {
    AlgaeRollersIOSparkMax sparkMaxes = new AlgaeRollersIOSparkMax();
    SparkRelativeEncoderSim SparkSim = new SparkRelativeEncoderSim(sparkMaxes.getMotor());

    inputs.positionRotations = SparkSim.getPosition();
    inputs.velocityRPM = SparkSim.getVelocity();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = 0;
    // inputs.temperature = 0;
  }

  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
  }
}