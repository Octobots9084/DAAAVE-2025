package frc.robot.Subsystems.AlgaeRollers;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class AlgaeRollersIOSparkMax implements AlgaeRollersIO {
    // TODO Change device ID    
    private final SparkMax motor = new SparkMax(18, MotorType.kBrushless);
    // private double feedForward = 0;
    private SparkMaxConfig config;

    public AlgaeRollersIOSparkMax() {
        config = new SparkMaxConfig();
        config.inverted(false).idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionAlwaysOn(true);
        config.signals.primaryEncoderPositionPeriodMs(10);
        config.voltageCompensation(11);
        config.smartCurrentLimit(30, 60);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.setPeriodicFrameTimeout(30);
        motor.setCANTimeout(30);
        motor.setCANMaxRetries(5);
    }

    public void updateInputs(AlgaeRollersIOInputs inputs) {
        inputs.velocityRPM = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public double getVoltage() {
        return motor.getBusVoltage();
    }

    public SparkMax getMotor() {
        return motor;
    }
}
