package frc.robot.Subsystems.Climb;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

        sparkConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder).pid(30, 0.0001, 0, ClosedLoopSlot.kSlot0);
        sparkConfig.closedLoop.pid(2, 0.00, 0, ClosedLoopSlot.kSlot1);

        sparkConfig.closedLoop.iZone(0.01);
        sparkConfig.closedLoop.iMaxAccum(3.5);
        sparkConfig.closedLoop.positionWrappingEnabled(true);
        sparkConfig.closedLoop.positionWrappingInputRange(0, 1);
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

        talonFXSConfig.CurrentLimits.SupplyCurrentLimit = 40;
        talonFXSConfig.CurrentLimits.SupplyCurrentLimitEnable = true;

        talonFXSConfig.CurrentLimits.StatorCurrentLimit = 40;
        talonFXSConfig.CurrentLimits.StatorCurrentLimitEnable = true;

        SmartDashboard.putString("ClimbTalonError", talonFXS.getConfigurator().apply(talonFXSConfig).toString());

    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        inputs.positionRotations = this.getPosition();
        inputs.velocityRPM = sparkMax.getAbsoluteEncoder().getVelocity();
        inputs.appliedVolts = sparkMax.getAppliedOutput();
        inputs.busVoltage = sparkMax.getBusVoltage();
        inputs.currentAmps = sparkMax.getOutputCurrent();
        SmartDashboard.putNumber("ClimbTalonSupplyCurrent", talonFXS.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("ClimbTalonStatorCurrent", talonFXS.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putBoolean("ClimbTalonStallCurrent", talonFXS.getFault_StatorCurrLimit().getValue());

    }

    @Override
    public double getPosition() {
        return sparkMax.getAbsoluteEncoder().getPosition();
    }

    @Override
    public void setPosition(double newPosition, ClosedLoopSlot slot) {
        sparkMax
                .getClosedLoopController()
                .setReference(newPosition, ControlType.kPosition, slot, feedForward);
    }

    @Override
    public void allStop() {
        sparkMax.setVoltage(0);

    }

    @Override
    public void setVoltage(double voltage) {
        sparkMax.setVoltage(voltage);
    }

    @Override
    public void setTalonVoltage(double voltage) {
        talonFXS.setVoltage(voltage);
    }

    @Override
    public boolean talonIsStalled() {
        return talonFXS.getFault_StatorCurrLimit().getValue();
    }
}