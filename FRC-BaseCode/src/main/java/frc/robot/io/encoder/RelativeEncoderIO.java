package frc.robot.io.encoder;

import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;

public class RelativeEncoderIO implements EncoderIO {
    private RelativeEncoder encoder;
    
    public RelativeEncoderIO(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Timer.getFPGATimestamp();
        
        values.positionRad.update(Units.rotationsToRadians(encoder.getPosition()), timestampNow);
        values.velocityRadPerSec.update(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity()), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void setPosition(double positionRad) {
        encoder.setPosition(Units.radiansToRotations(positionRad));
    }
}
