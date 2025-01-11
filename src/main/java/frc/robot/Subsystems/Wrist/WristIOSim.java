package frc.robot.Subsystems.Wrist;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.MathUtil;

public class WristIOSim implements WristIO {
  WristIOSparkMax sparkMaxes = new WristIOSparkMax();
  SparkRelativeEncoderSim wristSparkSim = new SparkRelativeEncoderSim(sparkMaxes.getWristMotor());
  private double appliedVolts = 0;

  @Override
  public void updateInputs(WristIOInputs inputs) {

    inputs.wristPositionRotations = wristSparkSim.getPosition();
    inputs.wristVelocityRPM = wristSparkSim.getVelocity();
    inputs.wristAppliedVolts = appliedVolts;
    inputs.wristCurrentAmps = 0;
    inputs.wristTemperature = 0;
  }

  public void setVoltage(double volts) {
    appliedVolts = MathUtil.clamp(volts, -7.0, 7.0);
  }
}
