package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RangeAlignSource extends SubsystemBase {
    CANrange backRange;
    PIDController backRangePID;
    CANrangeConfiguration configuration;
    FovParamsConfigs paramsConfigs;

    private static RangeAlignSource instance;

    public static RangeAlignSource getInstance() {
        if (instance == null) {
            instance = new RangeAlignSource();
        }
        return instance;
    }

    public RangeAlignSource() {
        this.backRange = new CANrange(0, "KrakensBus");
        this.backRangePID = new PIDController(2, 0, 0);

        this.paramsConfigs = new FovParamsConfigs();
        paramsConfigs.withFOVRangeX(6.75);
        paramsConfigs.withFOVRangeY(6.75);
        paramsConfigs.withFOVCenterX(6.75);
        paramsConfigs.withFOVCenterY(6.75);

        this.configuration = new CANrangeConfiguration();
        configuration.withFovParams(paramsConfigs);
        configuration.ProximityParams.ProximityThreshold = 1;
        backRange.getConfigurator().apply(configuration);
    }

    public double getBackRange() {
        return backRange.getDistance().getValueAsDouble();
    }

    public ChassisSpeeds getAlignChassisSpeeds() {
        return new ChassisSpeeds(backRangePID.calculate(this.getBackRange()), 0, 0);
    }
}
