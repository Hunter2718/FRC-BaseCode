package frc.robot.subsystems.drive;

import frc.robot.io.encoder.EncoderIO;
import frc.robot.io.motor.MotorGroup;

public class SwerveModule {

    private MotorGroup drive;
    private MotorGroup turn;
    private EncoderIO encoder;
    
    public SwerveModule(
        MotorGroup drive,
        MotorGroup turn,
        EncoderIO encoder
    ) {
        this.drive = drive;
        this.turn = turn;
        this.encoder = encoder;
    }

    public static double wrapRad(double rad) {
        while (rad > Math.PI) rad -= 2.0 * Math.PI;
        while (rad < -Math.PI) rad += 2.0 * Math.PI;
        return rad;
    }

    public static SwerveState optimize(double desiredSpeedMps, double desiredAngleRad, double currentAngleRad) {
        double error = wrapRad(desiredAngleRad - currentAngleRad);

        if (Math.abs(error) > (Math.PI / 2.0)) {
            // flip drive direction to reduce steering travel
            return new SwerveState(-desiredSpeedMps, wrapRad(desiredAngleRad + Math.PI));
        }
        return new SwerveState(desiredSpeedMps, wrapRad(desiredAngleRad));
    }
}
