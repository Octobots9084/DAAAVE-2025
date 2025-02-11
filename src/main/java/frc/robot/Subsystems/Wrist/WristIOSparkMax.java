package frc.robot.Subsystems.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.CoralRollers.CoralRollers;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class WristIOSparkMax implements WristIO {
  // TODO - motor id to be changed
  private final SparkMax wristMotor = new SparkMax(12, MotorType.kBrushless);
  private double offset = 0;
  // private double feedForward = 0;

  private SparkMaxConfig config;

  public WristIOSparkMax() {
    config = new SparkMaxConfig();
    config.inverted(false);
    config.idleMode(IdleMode.kBrake);
    config.voltageCompensation(3);
    config.smartCurrentLimit(40, 40);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(1, 0.0, 0, ClosedLoopSlot.kSlot0);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(7, 0.000, 0, ClosedLoopSlot.kSlot1);

    wristMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    wristMotor.setPeriodicFrameTimeout(30);
    wristMotor.setCANTimeout(30);
    wristMotor.setCANMaxRetries(5);
  }

  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPositionRotations = wristMotor.getAbsoluteEncoder().getPosition();
    inputs.wristVelocityRPM = wristMotor.getEncoder().getVelocity();
    inputs.wristAppliedVolts = wristMotor.getAppliedOutput();
    inputs.wristBusVoltage = wristMotor.getBusVoltage();
    inputs.wristCurrentAmps = wristMotor.getOutputCurrent();
    // double ffVal = 0.5 * (Math.sin((this.getPosition() - 0.113) * 2 * Math.PI));
    // SmartDashboard.putNumber("wristFeedForward", ffVal);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    config.closedLoop.pid(kP, kI, kD);

  }

  @Override
  public SparkMax getWristMotor() {
    return wristMotor;
  }

  @Override
  public void setOffset(double offset) {
    this.offset = offset;
  }

  @Override
  public void setPosition(double position, ClosedLoopSlot slot) {
    SmartDashboard.putNumber("commandedWristPosition", position);
    double ffVal = 0.8 * (Math.sin((position - 0.113) * 2 * Math.PI));
    SmartDashboard.putNumber("wristFeedForward", ffVal);

    if (CoralRollers.getInstance().io.HasCoral()) {
      wristMotor
          .getClosedLoopController()
          .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot1, ffVal);
    } else {
      wristMotor
          .getClosedLoopController()
          .setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVal);
    }
  }

  @Override
  public double getPosition() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }
}
