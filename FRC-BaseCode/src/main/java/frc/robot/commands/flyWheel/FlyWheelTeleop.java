package frc.robot.commands.flyWheel;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flyWheel.FlyWheelSubsystem;

public class FlyWheelTeleop extends Command {
    private FlyWheelSubsystem flywheel;

    private Supplier<Boolean> spinSup;
    private Supplier<Boolean> lowSup;
    private Supplier<Boolean> highSup;

    private double kLowRadPerSec;
    private double kHighRadPerSec;
    private double kNormalRadPerSec;

    public FlyWheelTeleop(
        FlyWheelSubsystem flywheel,
        Supplier<Boolean> spinSup,
        Supplier<Boolean> lowSup,
        Supplier<Boolean> highSup,
        double kLowRadPerSec,
        double kHighRadPerSec,
        double kNormalRadPerSec
    ) {
        this.flywheel = flywheel;
        this.spinSup = spinSup;
        this.lowSup = lowSup;
        this.highSup = highSup;
        this.kLowRadPerSec = kLowRadPerSec;
        this.kHighRadPerSec = kHighRadPerSec;
        this.kNormalRadPerSec = kNormalRadPerSec;

        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        if (!spinSup.get()) {
            flywheel.stop();
            return;
        }

        double target = kNormalRadPerSec;
        if (lowSup.get() && !highSup.get()) {
            target = kLowRadPerSec;
        } else if (highSup.get() && !lowSup.get()) {
            target = kHighRadPerSec;
        }

        flywheel.setTargetWheelRadPerSec(target);
    }

    @Override
    public void end(boolean interrupted) {
        flywheel.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
