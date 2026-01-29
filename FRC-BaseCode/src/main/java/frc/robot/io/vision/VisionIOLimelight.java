package frc.robot.io.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.utils.LimelightHelpers;

public class VisionIOLimelight implements VisionIO {
    public enum Mode {
        MEGATAG1_WPIBLUE,
        MEGATAG2_WPIBLUE
    }

    private final String limelightName;
    private final Mode mode;
    private final Matrix<N3, N1> stdDevs;

    private Rotation3d lastRobotOrientation = new Rotation3d();

    public VisionIOLimelight(String limelightName, Mode mode, Matrix<N3, N1> stdDevs) {
        this.limelightName = limelightName;
        this.mode = mode;
        this.stdDevs = stdDevs;
    }

    @Override
    public void setRobotOrientation(Rotation3d robotOrientation) {
        this.lastRobotOrientation = robotOrientation;
    }

    @Override
    public void updateInputs(VisionIOValues values) {
        values.name = limelightName;
        values.poseMeasurements.clear();
        values.hasPose = false;

        // Limelight Lib explicitly says MegaTag2 requires SetRobotOrientation() beforehand
        if (mode == Mode.MEGATAG2_WPIBLUE) {
            double yawDeg = Math.toDegrees(lastRobotOrientation.getZ());
            LimelightHelpers.SetRobotOrientation(limelightName, yawDeg, 0.0, 0.0, 0.0, 0.0, 0.0);
        }

        LimelightHelpers.PoseEstimate pe =
            (mode == Mode.MEGATAG2_WPIBLUE)
                ? LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName) 
                : LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);         

        // PoseEstimate includes pose + timestampSeconds + tagCount + avgTagDist + rawFiducials, etc
        int tagCount = pe.tagCount;
        values.connected = true;            // Limelight Lib doesn’t expose a clean "connected" boolean in PoseEstimate.
        values.hasPose = tagCount > 0;

        if (!values.hasPose) return;

        double ambiguity = Double.NaN;
        if (pe.rawFiducials != null && pe.rawFiducials.length > 0) {
            double maxAmb = pe.rawFiducials[0].ambiguity;
            for (var f : pe.rawFiducials) {
                if (f.ambiguity > maxAmb) maxAmb = f.ambiguity;
            }
            ambiguity = maxAmb;
        }

        values.poseMeasurements.add(new VisionPoseMeasurement(
            pe.pose,
            pe.timestampSeconds,
            stdDevs,
            VisionSource.LIMELIGHT,
            tagCount,
            pe.avgTagDist,
            ambiguity
        ));
    }
}
