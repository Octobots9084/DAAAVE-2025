package frc.robot.Subsystems.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class WristIOSparkMax implements WristIO {
  // TODO - motor id to be changed
  private final SparkMax wristMotor = new SparkMax(17, MotorType.kBrushless);
  // private double feedForward = 0;

  private SparkMaxConfig config;

  public WristIOSparkMax() {
    config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.signals.primaryEncoderPositionAlwaysOn(true);
    config.signals.primaryEncoderPositionPeriodMs(10);
    config.voltageCompensation(0);
    config.smartCurrentLimit(30, 60);

    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.setPeriodicFrameTimeout(30);
    wristMotor.setCANTimeout(30);
    wristMotor.setCANMaxRetries(5);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPositionRotations = wristMotor.getEncoder().getPosition();
    inputs.wristVelocityRPM = wristMotor.getEncoder().getVelocity();
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput() * wristMotor.getBusVoltage();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);
  }

  @Override
  public void setPosition(double position) {
    wristMotor
        .getClosedLoopController()
        .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
  }
}
