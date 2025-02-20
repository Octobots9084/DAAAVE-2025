/*
 * MIT License Copyright (c) PhotonVision Permission is hereby granted, free of
 * charge, to any
 * person obtaining a copy of this software and associated documentation files
 * (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the
 * Software, and to permit
 * persons to whom the Software is furnished to do so, subject to the following
 * conditions: The
 * above copyright notice and this permission notice shall be included in all
 * copies or substantial
 * portions of the Software. THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY
 * OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
 * CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE
 * OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionCamera implements Runnable {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private Matrix<N3, N1> curStdDevs;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();
    private final AtomicReference<Matrix<N3, N1>> atomicStdDev = new AtomicReference<Matrix<N3, N1>>();
    private final AtomicReference<PhotonPipelineResult> atomicPhotonResult = new AtomicReference<PhotonPipelineResult>();

    public VisionCamera(String photonCameraName, Transform3d robotToCamera) {
        camera = new PhotonCamera(photonCameraName);

        photonEstimator = new PhotonPoseEstimator(
                VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);
        photonEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();

        for (var change : camera.getAllUnreadResults()) {
            atomicPhotonResult.set(change);
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }

        // if (!unreadResults.isEmpty()) {
        //     latestResult = unreadResults.get(unreadResults.size() - 1);
        //     atomicPhotonResult.set(latestResult);
        //     visionEst = photonEstimator.update(latestResult);
        //     updateEstimationStdDevs(visionEst, latestResult.getTargets());
        // }

        return visionEst;
    }

    public boolean isConnected() {
        return camera.isConnected();
    }

    @Override
    public void run() {
        Optional<EstimatedRobotPose> estimatedRobotPose = getEstimatedGlobalPose();
        if (estimatedRobotPose.isPresent()) {
            EstimatedRobotPose pose = estimatedRobotPose.get();
            atomicEstimatedRobotPose.set(pose);
            atomicStdDev.set(curStdDevs);
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }

    public Matrix<N3, N1> grabLatestStdDev() {
        return atomicStdDev.getAndSet(null);
    }

    public PhotonPipelineResult grabLatestResult() {
        return atomicPhotonResult.get();
    }

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kMultiTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an
            // average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty())
                    continue;
                numTags++;
                avgDist += tagPose
                        .get()
                        .toPose2d()
                        .getTranslation()
                        .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1)
                    estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else
                    estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    public PhotonPoseEstimator getPoseEstimator() {
        return photonEstimator;
    }

    public PhotonCamera getCamera() {
        return camera;
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
        return curStdDevs;
    }
}
