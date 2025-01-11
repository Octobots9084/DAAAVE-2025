package frc.robot.Subsystems.Elevator;

import com.revrobotics.sim.SparkRelativeEncoderSim;
import edu.wpi.first.math.MathUtil;

public class ElevatorIOSim implements ElevatorIO {

    private double appliedVolts = 11;

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        ElevatorIOSparkMax sparkMaxes = new ElevatorIOSparkMax();
        SparkRelativeEncoderSim leftSparkSim = new SparkRelativeEncoderSim(sparkMaxes.getLeftMotor());
        SparkRelativeEncoderSim rightSparkSim = new SparkRelativeEncoderSim(sparkMaxes.getRightMotor());

        inputs.leftPositionRotations = leftSparkSim.getPosition();
        inputs.leftVelocityRPM = leftSparkSim.getVelocity();
        inputs.leftAppliedVolts = appliedVolts;
        inputs.leftCurrentAmps = 0;
        inputs.leftTemperature = 0;

        inputs.rightPositionRotations = rightSparkSim.getPosition();
        inputs.rightVelocityRPM = rightSparkSim.getVelocity();
        inputs.rightAppliedVolts = appliedVolts;
        inputs.rightCurrentAmps = 0;
        inputs.rightTemperature = 0;
    }

    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
