package frc.robot.subsystems.vision;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.vision.VisionIO;
import frc.robot.io.vision.VisionIO.VisionIOValues;
import frc.robot.io.vision.VisionIO.VisionPoseMeasurement;
import frc.robot.io.vision.VisionIO.VisionSource;

public class VisionSubsystem extends SubsystemBase {
    private List<VisionIO> ios;
    private List<VisionIOValues> values;
    private Supplier<Rotation3d> robotOrientationSupplier;
    private Consumer<VisionPoseMeasurement> measurementConsumer;
    private Supplier<Pose2d> currentEstimatedPose;
    private List<Consumer<Pose2d>> alignQuestsToPose;

    private boolean questAligned;
    private double lastQuestAlignTime;

    public VisionSubsystem(
        List<VisionIO> ios,
        List<VisionIOValues> values,
        Supplier<Rotation3d> robotOrientationSupplier,
        Consumer<VisionPoseMeasurement> measurementConsumer,
        Supplier<Pose2d> currentEstimatedPose,
        List<Consumer<Pose2d>> alignQuestsToPose
    ) {
        this.ios = ios;
        this.values = values;
        this.robotOrientationSupplier = robotOrientationSupplier;
        this.measurementConsumer = measurementConsumer;
        this.currentEstimatedPose = currentEstimatedPose;
        this.alignQuestsToPose = alignQuestsToPose;

        questAligned = false;
        lastQuestAlignTime = -9999999;
    }

    @Override
    public void periodic() {
        Rotation3d rot = robotOrientationSupplier != null ? robotOrientationSupplier.get() : new Rotation3d();

        // Ignoring bad inputs
        boolean stable = isStable(rot);

        for (int i = 0; i < ios.size(); i++) {
            VisionIO io = ios.get(i);
            VisionIOValues v = values.get(i);

            io.setRobotOrientation(rot);

            v.poseMeasurements.clear();
            io.updateInputs(v);

            for (VisionPoseMeasurement m : v.poseMeasurements) {
                if(!stable) continue;

                if(!isMeasurementUsable(m)) continue;

                measurementConsumer.accept(m);

                if(shouldAlignQuest(m)) alignAllQuests(currentEstimatedPose.get());
            }
        }
    }

    public boolean isStable(Rotation3d rot) {
        double rollRad = rot.getX();
        double ptichRad = rot.getY();
        
        return Math.abs(rollRad) <= VisionSubsystemConstants.kMaxRollRadForFusion && Math.abs(ptichRad) <= VisionSubsystemConstants.kMaxPitchRadForFusion;
    }

    public boolean isMeasurementUsable(VisionPoseMeasurement m) {
        Pose2d est = currentEstimatedPose.get();

        // Dont allow teleport jumps
        double jump = m.pose().getTranslation().getDistance(est.getTranslation());
        if(jump > VisionSubsystemConstants.kMaxVisionJumpMeters) return false;

        if(m.source() == VisionSource.LIMELIGHT || m.source() == VisionSource.PHOTONVISION) {
            if(m.tagCount() <= 0) return false;

            if(!Double.isNaN((m.avgTagDistMeters())) && m.avgTagDistMeters() > VisionSubsystemConstants.kMaxAvgTagDistMeters) return false;

            if(!Double.isNaN(m.ambiguity()) && m.ambiguity() > VisionSubsystemConstants.kMaxAmbiguity) return false;
        }

        return true;
    }

    private boolean shouldAlignQuest(VisionPoseMeasurement m) {
        // Only align off "absolute" sources
        if (!(m.source() == VisionSource.LIMELIGHT || m.source() == VisionSource.PHOTONVISION)) return false;

        // Only align when this measurement is "good enough"
        boolean good =
            m.tagCount() >= 2 ||
            (m.tagCount() == 1
                && (Double.isNaN(m.avgTagDistMeters()) || m.avgTagDistMeters() < VisionSubsystemConstants.kMaxAvgTagDistMeters)
                && (Double.isNaN(m.ambiguity()) || m.ambiguity() < VisionSubsystemConstants.kMaxAmbiguity));

        if (!good) return false;

        double now = Timer.getFPGATimestamp();
        if (questAligned && (now - lastQuestAlignTime) < VisionSubsystemConstants.kQuestAlignMinPeriodSec) return false;

        Pose2d estPose = currentEstimatedPose.get();

        // If already aligned, only realign if we drifted noticeably (unless user requested)
        if (questAligned) {
            double err = estPose.getTranslation().getDistance(m.pose().getTranslation());
            if (err < VisionSubsystemConstants.kQuestAlignErrorMeters) return false;
        }

        return true;
        
    }

    private void alignAllQuests(Pose2d pose) {
        double now = Timer.getFPGATimestamp();

        for(Consumer<Pose2d> quest : alignQuestsToPose) {
            quest.accept(pose);
        }

        questAligned = true;
        lastQuestAlignTime = now;
    }
}
