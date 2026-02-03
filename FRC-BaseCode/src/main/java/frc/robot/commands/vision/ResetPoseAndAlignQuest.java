package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.vision.VisionSubsystem;

public class ResetPoseAndAlignQuest extends InstantCommand {
    public ResetPoseAndAlignQuest(
        DriveSubsystem drive,
        VisionSubsystem vision,
        Pose2d pose
    ) {
        super(
            () -> {
                drive.resetPose(pose);
                vision.forceAlignAllQuests(pose);
            },
            drive,
            vision
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
