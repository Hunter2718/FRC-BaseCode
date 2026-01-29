package frc.robot.io.vision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionIOPhotonVision implements VisionIO {
    private String name;
    private PhotonCamera camera;
    private PhotonPoseEstimator estimator;
    private AprilTagFieldLayout fieldLayout;

    private Matrix<N3, N1> singleTagStdDevs;
    private Matrix<N3, N1> multiTagStdDevs;

    private double rejectIfAvgTagDistOverMeters;


    public VisionIOPhotonVision(
        String name,
        PhotonCamera camera,
        AprilTagFieldLayout fieldLayout,
        Transform3d robotToCamera,
        Matrix<N3, N1> singleTagStdDevs,
        Matrix<N3, N1> multiTagStdDevs,
        double rejectIfAvgTagDistOverMeters
    ) {
        this.name = name;
        this.camera = camera;
        this.fieldLayout = fieldLayout;

        this.estimator = new PhotonPoseEstimator(fieldLayout, PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);

        this.singleTagStdDevs = singleTagStdDevs;
        this.multiTagStdDevs = multiTagStdDevs;
        this.rejectIfAvgTagDistOverMeters = rejectIfAvgTagDistOverMeters;
    }

    @Override
    public void updateInputs(VisionIOValues values) {
        values.name = name;
        values.connected = camera.isConnected();
        values.hasPose = false;
        values.poseMeasurements.clear();

        // Should be called once per loop; returns all unread pipeline results
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        for (PhotonPipelineResult result : results) {
            if (!result.hasTargets()) continue;

            Optional<EstimatedRobotPose> est = estimate(result);
            if (est.isEmpty()) continue;

            EstimatedRobotPose erp = est.get(); // has estimatedPose, timestampSeconds, targetsUsed
            Pose2d pose2d = erp.estimatedPose.toPose2d();
            double ts = erp.timestampSeconds;

            TagStats stats = computeTagStats(erp.estimatedPose, erp.targetsUsed);
            if (rejectIfAvgTagDistOverMeters > 0 && stats.avgDistMeters > rejectIfAvgTagDistOverMeters) {
                continue;
            }

            Matrix<N3, N1> std = (stats.tagCount >= 2) ? multiTagStdDevs : singleTagStdDevs;

            values.poseMeasurements.add(new VisionPoseMeasurement(
                pose2d,
                ts,
                std,
                VisionSource.PHOTONVISION,
                stats.tagCount,
                stats.avgDistMeters,
                stats.maxAmbiguity
            ));
            values.hasPose = true;
        }
    }

    private Optional<EstimatedRobotPose> estimate(PhotonPipelineResult result) {
        // Multi-tag coprocessor pose if you have it enabled in PhotonVision UI
        if (result.targets.size() >= 2) {
            Optional<EstimatedRobotPose> mt = estimator.estimateCoprocMultiTagPose(result);
            if (mt.isPresent()) return mt;
        }
        // Fallback to single-tag lowest ambiguity
        return estimator.estimateLowestAmbiguityPose(result);
    }

    private TagStats computeTagStats(Pose3d estimatedRobotPose, List<PhotonTrackedTarget> targetsUsed) {
        int count = 0;
        double distSum = 0.0;
        double maxAmb = Double.NaN;

        for (PhotonTrackedTarget t : targetsUsed) {
            int id = t.getFiducialId();
            var tagPoseOpt = fieldLayout.getTagPose(id);
            if (tagPoseOpt.isEmpty()) continue;

            Pose3d tagPose = tagPoseOpt.get();
            double dist = tagPose.getTranslation().getDistance(estimatedRobotPose.getTranslation());
            distSum += dist;
            count++;

            double amb = t.getPoseAmbiguity();
            if (Double.isNaN(maxAmb) || amb > maxAmb) maxAmb = amb;
        }

        double avgDist = (count > 0) ? (distSum / count) : 0.0;
        return new TagStats(count, avgDist, maxAmb);
    }

    private static class TagStats {
        final int tagCount;
        final double avgDistMeters;
        final double maxAmbiguity;

        TagStats(int tagCount, double avgDistMeters, double maxAmbiguity) {
            this.tagCount = tagCount;
            this.avgDistMeters = avgDistMeters;
            this.maxAmbiguity = maxAmbiguity;
        }
    }
}
