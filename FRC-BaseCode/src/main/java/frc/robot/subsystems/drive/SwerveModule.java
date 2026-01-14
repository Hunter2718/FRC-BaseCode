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
        return Math.atan2(Math.sin(rad), Math.cos(rad));
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
