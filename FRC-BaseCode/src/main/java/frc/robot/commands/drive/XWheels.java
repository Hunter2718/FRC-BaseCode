package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.DriveSubsystem;

public class XWheels extends InstantCommand {
    public XWheels(
        DriveSubsystem drive
    ) {
        super(
            () -> {
                drive.setX();
            },
            drive
        );
    }
}
