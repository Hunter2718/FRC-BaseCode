package frc.robot.io.vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public interface VisionIO {

    public enum VisionSource {
        PHOTONVISION,
        LIMELIGHT,
        QUESTNAV,
        OTHER
    }

    /** A single pose measurement appropriate for addVisionMeasurement(). */
    public record VisionPoseMeasurement(
        Pose2d pose,
        double timestampSeconds,
        Matrix<N3, N1> stdDevs,
        VisionSource source,
        int tagCount,
        double avgTagDistMeters,
        double ambiguity
    ) {}

    public class VisionIOValues {
        public String name = "unknown";
        public boolean connected = false;
        public boolean hasPose = false;

        /** New pose measurements since last updateInputs() */
        public final List<VisionPoseMeasurement> poseMeasurements = new ArrayList<>();
    }

    /** Fill values. Implementations should clear values.poseMeasurements and then add new ones. */
    default void updateInputs(VisionIOValues values) {}

    /**
     * Optional hook for implementations that need robot orientation (ex: Limelight MegaTag2).
     * Rotation3d is radians.
     */
    default void setRobotOrientation(Rotation3d robotOrientation) {}
}