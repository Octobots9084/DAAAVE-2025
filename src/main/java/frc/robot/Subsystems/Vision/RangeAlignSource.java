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
    boolean rotInTolerance = false;

    private static RangeAlignSource instance;

    public static RangeAlignSource getInstance() {
        if (instance == null) {
            instance = new RangeAlignSource();
        }
        return instance;
    }

    public RangeAlignSource() {
        this.backRange = new CANrange(23, "KrakensBus");
        this.backRangePID = new PIDController(3, 0, 0);
        this.gyroRotationPIDController = new PIDController(0.8, 0, 0);
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

    // From Nate :)
    // This should return whether the robot is close enough to the source to take control
    // from the drivers and align.
    public boolean wrenchControlFromDriversForSourceAlign() {
        return backRange.getIsDetected().getValue();
    }

    public ChassisSpeeds getAlignChassisSpeeds() {
        double poseAngle = Swerve.getInstance().getPose().getRotation().getRadians();
        double sourceAngle = Math.toRadians(AlignVision.getInstance().handleTurnAngle(AlignState.SourceRight));
        double turnAngle = gyroRotationPIDController.calculate(poseAngle, sourceAngle);
        rotInTolerance = MathUtil.isNear(poseAngle, Math.toRadians(turnAngle), 0.05);

        if (rotInTolerance) {
            turnAngle = 0;
        }

        // only go back if we are within 1 meter
        double desiredSpeed = 0;
        if(this.backRange.getIsDetected().getValue()) {
            double distance = this.getBackRange();
            desiredSpeed = backRangePID.calculate(distance, 0.55);
        }

        return new ChassisSpeeds(desiredSpeed, 0, turnAngle);
    }
}
