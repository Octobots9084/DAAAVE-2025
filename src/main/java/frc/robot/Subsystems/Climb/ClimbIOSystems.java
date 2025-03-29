package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.AdvancedHallSupportValue;
import com.ctre.phoenix6.signals.BrushedMotorWiringValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.hal.FRCNetComm.tInstances;

import com.revrobotics.spark.config.SparkMaxConfig;

public class ClimbIOSystems implements ClimbIO {
    // TODO Change device ID
    private final SparkMax sparkMax = new SparkMax(14, MotorType.kBrushless);
    private final TalonFXS talonFXS = new TalonFXS(16, "KrakensBus");

    private SparkMaxConfig sparkConfig;
    private TalonFXSConfiguration talonFXSConfig;
    private double feedForward = 0;

    public ClimbIOSystems() {
        sparkConfig = new SparkMaxConfig();

        sparkConfig.inverted(false);
        sparkConfig.idleMode(IdleMode.kBrake);
        sparkConfig.signals.primaryEncoderPositionAlwaysOn(true);
        sparkConfig.signals.primaryEncoderPositionPeriodMs(10);
        sparkConfig.voltageCompensation(10);
        sparkConfig.smartCurrentLimit(60, 60);

        sparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(0.05, 0.0, 0, ClosedLoopSlot.kSlot0);

        sparkMax.configure(sparkConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        sparkMax.setPeriodicFrameTimeout(30);
        sparkMax.setCANTimeout(30);
        sparkMax.setCANMaxRetries(5);

        talonFXSConfig = new TalonFXSConfiguration();
        talonFXSConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        talonFXSConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        talonFXSConfig.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;

        talonFXSConfig.Voltage.PeakForwardVoltage = 10;
        talonFXSConfig.Voltage.PeakReverseVoltage = 10;

        talonFXSConfig.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXSConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXSConfig.CurrentLimits.StatorCurrentLimit = 30;
        talonFXSConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        talonFXS.getConfigurator().apply(talonFXSConfig);

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = sparkMax.getEncoder().getPosition();
        inputs.velocityRPM = sparkMax.getEncoder().getVelocity();
        inputs.appliedVolts = sparkMax.getAppliedOutput();
        inputs.busVoltage = sparkMax.getBusVoltage();
        inputs.currentAmps = sparkMax.getOutputCurrent();
    }

    @Override
    public double getPosition() {
        return sparkMax.getEncoder().getPosition();
    }

    @Override
    public void setPosition(double newPosition) {
        sparkMax
                .getClosedLoopController()
                .setReference(newPosition, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void zeroEncoder() {
        sparkMax.getEncoder().setPosition(0);
    }

    @Override
    public void setVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    @Override
    public void setTalonVoltage(double voltage) {
        talonFXS.setVoltage(voltage);
    }
}