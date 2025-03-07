package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.Warnings;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralRollersIOSystems implements CoralRollersIO {
    private final SparkFlex motor = new SparkFlex(13, MotorType.kBrushless);
    private CANrange clawBackSensor = new CANrange(21, "KrakensBus");
    private CANrange clawFrontSensor = new CANrange(22, "KrakensBus");
    private CANrangeConfiguration configuration = new CANrangeConfiguration();

    private SparkMaxConfig config;

    public CoralRollersIOSystems() {
        config = new SparkMaxConfig();
        config.inverted(false);
        config.idleMode(IdleMode.kBrake);
        config.smartCurrentLimit(40, 10);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motor.setPeriodicFrameTimeout(30);
        motor.setCANTimeout(30);
        motor.setCANMaxRetries(5);
        configuration.ProximityParams.withProximityThreshold(0.2);
        clawBackSensor.getConfigurator().apply(configuration);
        clawFrontSensor.getConfigurator().apply(configuration);
    }

    @Override
    public void updateInputs(CoralRollersIOInputs inputs) {
        inputs.velocityRPM = motor.getEncoder().getVelocity();
        inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.currentAmps = motor.getOutputCurrent();
        // TODO change 100
        inputs.isIntaking = this.IsIntaking();
        inputs.hasCoral = this.HasCoral();
        inputs.temperature = motor.getMotorTemperature();

        inputs.coralMeasureDist = clawFrontSensor.getDistance().getValueAsDouble();
    }

    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void rotateBy(double movement) {
        // TODO - make a PID controller
        motor.getClosedLoopController().setReference(motor.getAbsoluteEncoder().getPosition() + movement,
                ControlType.kPosition);
    }

    public void getVoltage(double voltage) {
        motor.getBusVoltage();
    }

    // TODO - Actually change these values
    @Override
    public boolean IsIntaking() {
        boolean intaking = clawBackSensor.getIsDetected().getValue();
        return intaking;
    }

    @Override
    public boolean HasCoral() {
        boolean coral = clawBackSensorTriggered() || clawFrontSensorTriggered();
        // SmartDashboard.putBoolean("HasCoral", coral);
        return coral;
    }

    @Override
    public boolean clawFrontSensorTriggered() {
        return clawFrontSensor.getDistance().getValueAsDouble() < 0.07;
    }

    @Override
    public boolean clawBackSensorTriggered() {
        return clawBackSensor.getDistance().getValueAsDouble() < 0.07;
    }

    @Override
    public boolean isStalled() {
        return motor.getOutputCurrent() > 25;
    }

}
