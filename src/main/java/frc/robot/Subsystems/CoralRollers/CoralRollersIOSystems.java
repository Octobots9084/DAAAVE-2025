package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;

public class CoralRollersIOSystems implements CoralRollersIO {
  private final SparkMax motor = new SparkMax(14, MotorType.kBrushless);
  private final AnalogInput mouthBeam = new AnalogInput(2);
  public CANrange clawFrontSensor = new CANrange(0);
  public CANrange clawBackSensor = new CANrange(0);

  private SparkMaxConfig config;

  public CoralRollersIOSystems() {
    config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30, 60);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.setPeriodicFrameTimeout(30);
    motor.setCANTimeout(30);
    motor.setCANMaxRetries(5);
  }

  @Override
  public void updateInputs(CoralRollersIOInputs inputs) {
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    // TODO change 100
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void getVoltage(double voltage) {
    motor.getBusVoltage();
  }

  // TODO - Actually change these values
  public boolean coralInChute() {
    return this.mouthBeam.getValue() > 100;
  }

  public boolean clawFrontSensorTriggered() {
    return clawFrontSensor.getDistance().getValueAsDouble() < 0.05;
  }

  public boolean clawBackSensorTriggered() {
    return clawBackSensor.getDistance().getValueAsDouble() < 0.05;
  }
}
