package frc.robot.Subsystems.Vision;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.States.AlignState;
import frc.robot.Subsystems.Swerve.Swerve;

public class AlignSourceAuto extends SubsystemBase {
    Swerve swerve;
    AlignVision alignVision;
    PIDController backRangePID;
    private PIDController gyroRotationPIDController;
    boolean rotInTolerance = false;

    private static AlignSourceAuto instance;

    public static AlignSourceAuto getInstance() {
        if (instance == null) {
            instance = new AlignSourceAuto();
        }
        return instance;
    }

    public AlignSourceAuto() {
        this.swerve = Swerve.getInstance();
        this.alignVision = AlignVision.getInstance();
        this.backRangePID = new PIDController(3, 0, 0);
        this.gyroRotationPIDController = new PIDController(0.8, 0, 0);
        this.gyroRotationPIDController.enableContinuousInput(0, 2 * Math.PI);
    }

    public double getBackRange() {
        return alignVision.getBackLidarDistance();
    }

    // From Nate :)
    // This should return whether the robot is close enough to the source to take
    // control
    // from the drivers and align.
    public boolean wrenchControlFromDriversForSourceAlign() {
        return alignVision.getBackLidarDistance() < 2.0;
    }

    public ChassisSpeeds getAlignChassisSpeeds() {
        double poseAngle = Swerve.getInstance().getPose().getRotation().getRadians();

        // if (!Constants.isBlueAlliance) {
        // distanceToLeftSide = swerve.getPose().getTranslation().getDistance(new
        // Translation2d(14.225, 4.7));
        // distanceToRightSide = swerve.getPose().getTranslation().getDistance(new
        // Translation2d(14.225, 3.35));
        // } else {
        // distanceToLeftSide = swerve.getPose().getTranslation().getDistance(new
        // Translation2d(3.325, 3.35));
        // distanceToRightSide = swerve.getPose().getTranslation().getDistance(new
        // Translation2d(3.325, 4.7));
        // }

        double sourceAngle = Math.toRadians(alignVision.handleTurnAngle(alignVision.getAlignSourceSide()));
        double turnAngle = gyroRotationPIDController.calculate(poseAngle, sourceAngle);
        rotInTolerance = MathUtil.isNear(poseAngle, Math.toRadians(turnAngle), 0.05);

        if (rotInTolerance) {
            turnAngle = 0;
        }

        // only go back if we are within 1 meter
        double desiredSpeed = 0;
        if (alignVision.getBackLidarDetect()) {
            double distance = this.getBackRange();
            desiredSpeed = backRangePID.calculate(distance, VisionConstants.maxBackLidarDepthDistance);
        }

        return new ChassisSpeeds(desiredSpeed, 0, turnAngle);
    }
}
