package frc.robot.commands.flyWheel;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flyWheel.FlyWheelSubsystem;

public class ConstantSpinToSpeed extends Command {
    private FlyWheelSubsystem flywheel;
    private Supplier<Double> goalWheelRadPerSec;

    public ConstantSpinToSpeed(
        FlyWheelSubsystem flywheel,
        Supplier<Double> goalWheelRadPerSec
    ) {
        this.flywheel = flywheel;
        this.goalWheelRadPerSec = goalWheelRadPerSec;

        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.setTargetWheelRadPerSec(goalWheelRadPerSec.get());
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
