package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.States.AlignState;
import frc.robot.Subsystems.Swerve.Swerve;

public class RangeAlignSource extends SubsystemBase {
    CANrange backRange;
    PIDController backRangePID;
    private PIDController gyroRotationPIDController;
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
        this.backRange = new CANrange(23, "KrakensBus");
        this.backRangePID = new PIDController(1, 0, 0);
        this.gyroRotationPIDController = new PIDController(0.4, 0, 0);
        this.gyroRotationPIDController.enableContinuousInput(0, 2 * Math.PI);

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
        // rotInTolerance = MathUtil.isNear(swerve.getGyro(), Math.toRadians(turnAngle), 0.05);

        return new ChassisSpeeds(backRangePID.calculate(this.getBackRange(),0.46), 0, gyroRotationPIDController.calculate(Swerve.getInstance().getGyro(), Math.toRadians(AlignVision.getInstance().handleTurnAngle(AlignState.Source))));
    }
}
