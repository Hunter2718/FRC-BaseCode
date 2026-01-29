package frc.robot.io.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

// must install questnav
import gg.questnav.questnav.QuestNav;
import gg.questnav.questnav.PoseFrame;

public class VisionIOQuestNav implements VisionIO {
    private final String name;
    private final QuestNav questNav;
    private final Transform3d robotToQuest;
    private final Matrix<N3, N1> stdDevs;

    public VisionIOQuestNav(
        String name,
        QuestNav questNav,
        Transform3d robotToQuest,
        Matrix<N3, N1> stdDevs
    ) {
        this.name = name;
        this.questNav = questNav;
        this.robotToQuest = robotToQuest;
        this.stdDevs = stdDevs;
    }

    @Override
    public void updateInputs(VisionIOValues values) {
        values.name = name;
        values.poseMeasurements.clear();
        values.hasPose = false;

        // QuestNav docs recommend calling commandPeriodic regularly :contentReference[oaicite:17]{index=17}
        questNav.commandPeriodic();

        values.connected = questNav.isConnected();
        if (!values.connected) return;

        PoseFrame[] frames = questNav.getAllUnreadPoseFrames(); // unread frames since last call :contentReference[oaicite:18]{index=18}
        for (PoseFrame f : frames) {
        if (!f.isTracking()) continue;

        Pose3d questPose = f.questPose3d();
        Pose3d robotPose3d = questPose.transformBy(robotToQuest.inverse());
        Pose2d robotPose2d = robotPose3d.toPose2d();

        // dataTimestamp is the timestamp you should use for pose estimator vision updates :contentReference[oaicite:19]{index=19}
        values.poseMeasurements.add(new VisionPoseMeasurement(
            robotPose2d,
            f.dataTimestamp(),
            stdDevs,
            VisionSource.QUESTNAV,
            0,
            0.0,
            Double.NaN
        ));
        values.hasPose = true;
        }
    }

    /** Reset QuestNav using a known robot pose (you must apply the mounting offset)*/
    public void resetFromRobotPose(Pose2d robotPose) {
        Pose3d robotPose3d = new Pose3d(robotPose);
        Pose3d questPose = robotPose3d.transformBy(robotToQuest);
        questNav.setPose(questPose);
    }
}
