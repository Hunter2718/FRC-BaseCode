package frc.robot.commands.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemConstants;
import frc.robot.subsystems.drive.DriveSubsystemConstants.DriveConstants.SpeedMode;

public class TeleopDrive extends Command {
    private DriveSubsystem m_drive;
    
    private Supplier<Double> xSup;
    private Supplier<Double> ySup;
    private Supplier<Double> rotSup;

    private Supplier<Boolean> fieldRelativeSup;
    private Supplier<Boolean> slowSup;
    private Supplier<Boolean> fastSup;


    private SlewRateLimiter xLimiter;
    private SlewRateLimiter yLimiter;
    private SlewRateLimiter rotLimiter;


    public TeleopDrive(
        DriveSubsystem m_drive,
        Supplier<Double> xSup,
        Supplier<Double> ySup,
        Supplier<Double> rotSup,
        Supplier<Boolean> fieldRelativeSup,
        Supplier<Boolean> slowSup,
        Supplier<Boolean> fastSup
    ) {

        this.m_drive = m_drive;
        this.xSup = xSup;
        this.ySup = ySup;
        this.rotSup = rotSup;
        this.fieldRelativeSup = fieldRelativeSup;
        this.slowSup = slowSup;
        this.fastSup = fastSup;

        this.xLimiter = new SlewRateLimiter(DriveSubsystemConstants.DriveConstants.kXSlewRateLimiterValue);
        this.yLimiter = new SlewRateLimiter(DriveSubsystemConstants.DriveConstants.kYSlewRateLimiterValue);
        this.rotLimiter = new SlewRateLimiter(DriveSubsystemConstants.DriveConstants.kRotSlewRateLimiterValue);

        addRequirements(m_drive);
    }


    private static SpeedMode getSpeedMode(boolean slow, boolean fast) {
        // If both pressed, prefer NORMAL to avoid ambiguity
        if (slow && !fast) return SpeedMode.SLOW;
        if (fast && !slow) return SpeedMode.FAST;
        return SpeedMode.NORMAL;
    }


    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double x = xSup.get();
        double y = ySup.get();
        double rot = rotSup.get();

        // Square inputs for finer low-speed control while keeping sign
        x = Math.copySign(x * x, x);
        y = Math.copySign(y * y, y);
        rot = Math.copySign(rot * rot, rot);

        // Slew limit for smoothness
        x = xLimiter.calculate(x);
        y = yLimiter.calculate(y);
        rot = rotLimiter.calculate(rot);

        SpeedMode speedMode = getSpeedMode(slowSup.get(), fastSup.get());

        double speedMultiplier = switch (speedMode) {
            case SLOW -> DriveSubsystemConstants.DriveConstants.kTeleopSlowMultiplier;
            case FAST -> DriveSubsystemConstants.DriveConstants.kTeleopFastMultiplier;
            default -> DriveSubsystemConstants.DriveConstants.kTeleopNormalMultiplier;
        };

        x *= speedMultiplier;
        y *= speedMultiplier;
        rot *= speedMultiplier;

        m_drive.drive(x, y, rot, fieldRelativeSup.get());
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
