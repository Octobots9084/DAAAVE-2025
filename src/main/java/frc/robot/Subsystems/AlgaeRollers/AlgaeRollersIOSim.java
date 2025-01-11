package frc.robot.Subsystems.AlgaeRollers;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.MathUtil;

public class AlgaeRollersIOSim implements AlgaeRollersIO {

    private double appliedVolts = 11;

    @Override
    public void updateInputs(AlgaeRollersIOInputs inputs) {
        AlgaeRollersIOSystems sparkMaxes = new AlgaeRollersIOSystems();
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
