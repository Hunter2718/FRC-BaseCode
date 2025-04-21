package frc.robot.Subsystems.Drive;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.IO.Gryo.GryoIO;

public class DriveSubsystem extends SubsystemBase {
    private ModuleObject flModule;
    private ModuleObject frModule;
    private ModuleObject blModule;
    private ModuleObject brModule;
    private GryoIO gryoIO;

    public DriveSubsystem(
        ModuleObject flModule,
        ModuleObject frModule,
        ModuleObject blModule,
        ModuleObject brModule,
        GryoIO gryoIO
    ) {
        this.flModule = flModule;
        this.frModule = frModule;
        this.blModule = blModule;
        this.brModule = brModule;
        this.gryoIO = gryoIO;
    }
}
