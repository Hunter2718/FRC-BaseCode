package frc.robot.commands.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignQuest extends InstantCommand {
    public AlignQuest(
        VisionSubsystem vision,
        Pose2d pose
    ) {
        super(
            () -> {
                vision.forceAlignAllQuests(pose);
            },
            vision
        );
    }

    @Override
    public boolean runsWhenDisabled() {
        return true;
    }
}
