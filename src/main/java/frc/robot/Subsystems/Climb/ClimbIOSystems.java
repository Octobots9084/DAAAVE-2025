package frc.robot.Subsystems.Climb;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbIOSystems implements ClimbIO {
    // TODO Change device ID
    private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
    // private double feedForward = 0;
    private SparkMaxConfig config;
    private double feedForward = 0.5;
  
    public ClimbIOSystems() {
        config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.signals.primaryEncoderPositionAlwaysOn(true);
        config.signals.primaryEncoderPositionPeriodMs(10);
        config.voltageCompensation(0); 
        config.smartCurrentLimit(30, 10);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.setPeriodicFrameTimeout(30);
        motor.setCANTimeout(30);
        motor.setCANMaxRetries(5);
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.encoderPosition = motor.getEncoder().getPosition();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    @Override
    public double getPosition() {
    return motor.getEncoder().getPosition();
    }
    @Override
    public void setPosition(double newPosition) {
    SmartDashboard.putNumber("position", newPosition);
    motor
        .getClosedLoopController()
        .setReference(newPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
  }

}