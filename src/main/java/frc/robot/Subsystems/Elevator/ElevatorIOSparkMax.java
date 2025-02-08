package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
  // TODO replace device ids with actual ones
  private final SparkMax leftMotor = new SparkMax(10, MotorType.kBrushless);
  private final SparkMax rightMotor = new SparkMax(11, MotorType.kBrushless);
  private double feedForward = 0.5;

  private SparkMaxConfig leftConfig;

  private SparkMaxConfig rightConfig;

  public ElevatorIOSparkMax() {
    leftConfig = new SparkMaxConfig();
    leftConfig.inverted(false);
    leftConfig.idleMode(IdleMode.kCoast);
    leftConfig.signals.primaryEncoderPositionAlwaysOn(true);
    leftConfig.signals.primaryEncoderPositionPeriodMs(10);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    leftConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
    leftConfig.closedLoop.positionWrappingEnabled(false);
    leftConfig.voltageCompensation(5);
    leftConfig.smartCurrentLimit(50, 10);
    leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0.000, 0);
    leftConfig.closedLoop.iZone(5);
    leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    leftMotor.setPeriodicFrameTimeout(30);
    leftMotor.setCANTimeout(30);
    leftMotor.setCANMaxRetries(5);
    leftMotor.getEncoder().setPosition(0);

    rightConfig = new SparkMaxConfig();
    rightConfig.inverted(false);
    rightConfig.idleMode(IdleMode.kCoast);
    rightConfig.signals.primaryEncoderPositionAlwaysOn(true);
    rightConfig.signals.primaryEncoderPositionPeriodMs(10);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    rightConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
    rightConfig.closedLoop.positionWrappingEnabled(false);
    rightConfig.voltageCompensation(5);
    rightConfig.smartCurrentLimit(50, 10);
    rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0.000, 0);
    rightConfig.closedLoop.iZone(5);
    rightMotor.configure(
        rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    rightMotor.setPeriodicFrameTimeout(30);
    rightMotor.setCANTimeout(30);
    rightMotor.setCANMaxRetries(5);
    rightMotor.getEncoder().setPosition(0);
  }

  @Override
  public SparkMax getLeftMotor() {
    return leftMotor;
  }
  
  @Override
  public SparkMax getRightMotor() {
    return rightMotor;
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftPositionRotations = leftMotor.getEncoder().getPosition(); // .getPosition();
    inputs.leftVelocityRPM = leftMotor.getEncoder().getVelocity();
    inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
    inputs.leftCurrentAmps = leftMotor.getOutputCurrent();

    inputs.rightPositionRotations = -rightMotor.getEncoder().getPosition();
    inputs.rightVelocityRPM = rightMotor.getEncoder().getPosition();
    inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    rightConfig.closedLoop.pid(kP, kI, kD);

    leftConfig.closedLoop.pid(kP, kI, kD);

  }

  @Override
  public void setPosition(double leftPosition, double rightPosition) {
    if (leftPosition < 0){
      leftPosition = 0;
    }
    if (rightPosition < 0){
      rightPosition = 0;
    }
    
    SmartDashboard.putNumber("position", rightPosition);
    leftMotor
        .getClosedLoopController()
        .setReference(leftPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    rightMotor
        .getClosedLoopController()
        .setReference(-rightPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, -feedForward);
  }

  @Override
  public double getPosition() {
    return leftMotor.getEncoder().getPosition();
  }
}
