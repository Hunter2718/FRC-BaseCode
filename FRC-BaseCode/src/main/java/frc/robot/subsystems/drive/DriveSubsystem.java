package frc.robot.subsystems.drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.gryo.GryoIO;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModule flModule;
    private SwerveModule frModule;
    private SwerveModule blModule;
    private SwerveModule brModule;
    private GryoIO gryoIO;

    public DriveSubsystem(
        SwerveModule flModule,
        SwerveModule frModule,
        SwerveModule blModule,
        SwerveModule brModule,
        GryoIO gryoIO
    ) {
        this.flModule = flModule;
        this.frModule = frModule;
        this.blModule = blModule;
        this.brModule = brModule;
        this.gryoIO = gryoIO;
    }
}
