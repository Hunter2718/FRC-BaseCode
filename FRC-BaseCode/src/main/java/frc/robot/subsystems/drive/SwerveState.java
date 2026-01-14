package frc.robot.subsystems.drive;


public class SwerveState {
    public double velocity;
    public double angle;

    public SwerveState(
        double velocity,
        double angle
    ) {
        this.velocity = velocity;
        this.angle = angle;
    }
}

