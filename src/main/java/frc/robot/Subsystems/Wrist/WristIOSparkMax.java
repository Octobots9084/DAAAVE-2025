package frc.robot.Subsystems.Wrist;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    config.inverted(true);
    config.idleMode(IdleMode.kCoast);
    config.voltageCompensation(0);
    config.smartCurrentLimit(60, 3);
    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    config.closedLoop.pid(1, 0, 0, ClosedLoopSlot.kSlot0);

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
  public void setOffset(double offset){
    this.offset = offset;
  }

  @Override
  public void setPosition(double position, ClosedLoopSlot slot) {
    SmartDashboard.putNumber("commandedWristPosition", position);
    wristMotor
        .getClosedLoopController()
        .setReference(position+offset, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0);
  }
  @Override
  public double getPosition() {
    return wristMotor.getAbsoluteEncoder().getPosition();
  }
}
