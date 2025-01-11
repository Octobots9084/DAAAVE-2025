package frc.robot.Subsystems.Elevator;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
    // TODO replace device ids with actual ones
    private final SparkMax leftMotor = new SparkMax(18, MotorType.kBrushless);
    private final SparkMax rightMotor = new SparkMax(19, MotorType.kBrushless);
    private double feedForward = 0;

    private SparkMaxConfig leftConfig;

    private SparkMaxConfig rightConfig;

    public ElevatorIOSparkMax() {
        leftConfig = new SparkMaxConfig();
        leftConfig.inverted(false).idleMode(IdleMode.kBrake);
        leftConfig.signals.primaryEncoderPositionAlwaysOn(true);
        leftConfig.signals.primaryEncoderPositionPeriodMs(10);
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        leftConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
        leftConfig.closedLoop.positionWrappingEnabled(false);
        leftConfig.voltageCompensation(11);
        leftConfig.smartCurrentLimit(30, 60);

        leftMotor.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        leftMotor.setPeriodicFrameTimeout(30);
        leftMotor.setCANTimeout(30);
        leftMotor.setCANMaxRetries(5);

        rightConfig = new SparkMaxConfig();
        rightConfig.inverted(false).idleMode(IdleMode.kBrake);
        rightConfig.signals.primaryEncoderPositionAlwaysOn(true);
        rightConfig.signals.primaryEncoderPositionPeriodMs(10);
        rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        rightConfig.closedLoop.maxMotion.allowedClosedLoopError(0);
        rightConfig.closedLoop.positionWrappingEnabled(false);
        rightConfig.voltageCompensation(11);
        rightConfig.smartCurrentLimit(30, 60);

        rightMotor.configure(
            rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        rightMotor.setPeriodicFrameTimeout(30);
        rightMotor.setCANTimeout(30);
        rightMotor.setCANMaxRetries(5);
    }

    public SparkMax getLeftMotor() {
        return leftMotor;
    }

    public SparkMax getRightMotor() {
        return rightMotor;
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.leftPositionRotations = leftMotor.getEncoder().getPosition(); // .getPosition();
        inputs.leftVelocityRPM = leftMotor.getEncoder().getVelocity();
        inputs.leftAppliedVolts = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.leftCurrentAmps = leftMotor.getOutputCurrent();

        inputs.rightPositionRotations = rightMotor.getEncoder().getPosition();
        inputs.rightVelocityRPM = rightMotor.getEncoder().getPosition();
        inputs.rightAppliedVolts = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
        inputs.rightCurrentAmps = rightMotor.getOutputCurrent();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        rightConfig.closedLoop.pid(0, 0, 0);

        leftConfig.closedLoop.pid(0, 0, 0);
    }

    @Override
    public void setPosition(double leftPosition, double rightPosition) {
        leftMotor
            .getClosedLoopController()
            .setReference(leftPosition, ControlType.kPosition, null, feedForward);
        rightMotor
            .getClosedLoopController()
            .setReference(rightPosition, ControlType.kPosition, null, feedForward);
    }
}
