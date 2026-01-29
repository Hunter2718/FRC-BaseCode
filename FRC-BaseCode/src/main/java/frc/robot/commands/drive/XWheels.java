package frc.robot.commands.drive;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;

public class XWheels extends Command {
    private DriveSubsystem m_drive;

    public XWheels(
        DriveSubsystem m_drive
    ) {
        this.m_drive = m_drive;

        addRequirements(m_drive);
    }

    @Override
    public void initialize() {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public void execute() {
        m_drive.setX();
    }

    @Override
    public void end(boolean interrupted) {
        m_drive.drive(0, 0, 0, true);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
