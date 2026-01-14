package frc.robot.io.encoder;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class CANcoderIO implements EncoderIO {
    private CANcoder encoder;

    public CANcoderIO(CANcoder encoder) {
        this.encoder = encoder;
    }

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Timer.getFPGATimestamp();
        values.positionRad.update(Units.rotationsToRadians(encoder.getPosition().getValueAsDouble()), timestampNow);
        values.velocityRadPerSec.update(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity().getValueAsDouble() * 60), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void setPosition(double positionRad) {
        encoder.setPosition(Units.radiansToRotations(positionRad));
    }
}
