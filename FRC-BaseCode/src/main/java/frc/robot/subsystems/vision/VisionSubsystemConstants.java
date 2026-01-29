package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;


public class VisionSubsystemConstants {
    public static final Vector<N3> llStd = VecBuilder.fill(0.7, 0.7, 1.0);
    public static final Vector<N3> pvSingleStd = VecBuilder.fill(0.9, 0.9, 1.2);
    public static final Vector<N3> pvMultiStd = VecBuilder.fill(0.4, 0.4, 0.7);
    public static final Vector<N3> questStd = VecBuilder.fill(0.03, 0.03, 0.05);

    public static final double kMaxPitchRadForFusion = Math.toRadians(10);
    public static final double kMaxRollRadForFusion = Math.toRadians(10);
    public static final double kMaxYawRateRadPerSecForAprilTagVision = 8.0;
    public static final double kMaxVisionJumpMeters = 2.0;
    public static final double kMaxAvgTagDistMeters = 3.0;
    public static final double kMaxAmbiguity = 0.15;
    public static final double kQuestAlignMinPeriodSec = 1.0;
    public static final double kQuestAlignErrorMeters = 0.75;

    public static final AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // Camera transforms (add more if more cameras)
    public static final Transform3d robotToLeftSideLL = new Transform3d();
    public static final Transform3d robotToRightSideLL = new Transform3d();
    public static final Transform3d robotToQuest = new Transform3d();
}
