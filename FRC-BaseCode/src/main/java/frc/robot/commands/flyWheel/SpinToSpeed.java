package frc.robot.commands.flyWheel;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.flyWheel.FlyWheelSubsystem;

public class SpinToSpeed extends Command {
    private FlyWheelSubsystem flywheel;
    private Supplier<Double> goalWheelRadPerSec;
    private int truthMotorIndex;

    public SpinToSpeed(
        FlyWheelSubsystem flywheel,
        Supplier<Double> goalWheelRadPerSec,
        int truthMotorIndex
    ) {
        this.flywheel = flywheel;
        this.goalWheelRadPerSec = goalWheelRadPerSec;
        this.truthMotorIndex = truthMotorIndex;

        addRequirements(flywheel);
    }

    @Override
    public void execute() {
        flywheel.setTargetWheelRadPerSec(goalWheelRadPerSec.get());
    }

    @Override
    public boolean isFinished() {
        return flywheel.atSpeed(truthMotorIndex);
    }
}
