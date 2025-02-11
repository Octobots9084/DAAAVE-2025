package frc.robot.Subsystems.CoralRollers;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CoralRollersIOSystems implements CoralRollersIO {
  private final SparkMax motor = new SparkMax(13, MotorType.kBrushless);
  private CANrange mouthBeam = new CANrange(21, "KrakensBus");
  private CANrange coralDetector = new CANrange(22, "KrakensBus");
  private CANrangeConfiguration configuration = new CANrangeConfiguration();

  private SparkMaxConfig config;

  public CoralRollersIOSystems() {
    config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.smartCurrentLimit(30, 10);
    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    motor.setPeriodicFrameTimeout(30);
    motor.setCANTimeout(30);
    motor.setCANMaxRetries(5);
    configuration.ProximityParams.withProximityThreshold(0.2);
    mouthBeam.getConfigurator().apply(configuration);
    coralDetector.getConfigurator().apply(configuration);
  }

  @Override
  public void updateInputs(CoralRollersIOInputs inputs) {
    inputs.velocityRPM = motor.getEncoder().getVelocity();
    inputs.appliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.currentAmps = motor.getOutputCurrent();
    // TODO change 100
    inputs.isIntaking = this.IsIntaking();
    inputs.hasCoral = this.HasCoral();

    inputs.coralMeasureDist = coralDetector.getDistance().getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    motor.setVoltage(voltage);
  }

  public void getVoltage(double voltage) {
    motor.getBusVoltage();
  }

  // TODO - Actually change these values
  @Override
  public boolean IsIntaking() {
    boolean intaking = mouthBeam.getIsDetected().getValue();
    SmartDashboard.putBoolean("IsIntaking", intaking);
    return intaking;
  }

  @Override
  public boolean HasCoral() {
    boolean coral = coralDetector.getIsDetected().getValue();
    SmartDashboard.putBoolean("HasCoral", coral);
    return coral;
  }
}
