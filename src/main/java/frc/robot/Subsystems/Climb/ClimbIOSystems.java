package frc.robot.Subsystems.Climb;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;


public class ClimbIOSystems implements ClimbIO {
    // TODO Change device ID
    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
    private SparkMaxConfig config;
    private double feedForward = 0;

    public ClimbIOSystems() {
        config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionAlwaysOn(true);
        config.signals.primaryEncoderPositionPeriodMs(10);
        config.voltageCompensation(10);
        config.smartCurrentLimit(60, 60);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0.0, 0, ClosedLoopSlot.kSlot0);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.setPeriodicFrameTimeout(30);
        motor.setCANTimeout(30);
        motor.setCANMaxRetries(5);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = motor.getEncoder().getPosition();
        inputs.velocityRPM = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput();
        inputs.busVoltage = motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
    }

    @Override
    public double getPosition() {
        return motor.getEncoder().getPosition();
    }

    @Override
    public void setPosition(double newPosition) {
        motor
                .getClosedLoopController()
                .setReference(newPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void zeroEncoder() {
        motor.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

}