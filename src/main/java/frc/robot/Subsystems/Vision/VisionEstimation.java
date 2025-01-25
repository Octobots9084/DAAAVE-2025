package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.Swerve.Swerve;

public class VisionEstimation extends SubsystemBase {
    private final Swerve swerve;
    public static VisionEstimation INSTANCE;

    public Matrix<N3, N1> defatultStdDev = VecBuilder.fill(0.5, 0.5, 99999);

    public final Vision centerEstimator = new Vision("CamOne", VisionConstants.camOneTransform);

    private final Notifier allNotifier = new Notifier(
            () -> {
                centerEstimator.run();
            });

    public VisionEstimation() {
        this.swerve = Swerve.getInstance();

        allNotifier.setName("runAll");
        allNotifier.startPeriodic(0.02);
    }

    public static VisionEstimation getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new VisionEstimation();
        }

        return INSTANCE;
    }

    @Override
    public void periodic() {
        if (VisionConstants.USE_VISION) {
            updatePose(centerEstimator);

        } else {
            allNotifier.close();
        }
    }

    public void updatePose(Vision estimator) {
        var cameraPose = estimator.grabLatestEstimatedPose();
        if (cameraPose != null) {
            var visionMeasurementStdDevs = estimator.grabLatestStdDev();

            var pose2d = cameraPose.estimatedPose.toPose2d();

            swerve.addVisionReading(pose2d, cameraPose.timestampSeconds, visionMeasurementStdDevs);
        }
    }
}
