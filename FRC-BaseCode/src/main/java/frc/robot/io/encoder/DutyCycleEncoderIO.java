package frc.robot.io.encoder;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;

public class DutyCycleEncoderIO implements EncoderIO{
    private DutyCycleEncoder encoder;
    private double resetEncoderValueRad;

    /**
     * 
     * @param encoder Pre-Configured absoluet encoder
     */
    public DutyCycleEncoderIO(DutyCycleEncoder encoder) {
        this.encoder = encoder;
        resetEncoderValueRad = 0.0;
    }
    

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Timer.getFPGATimestamp();
        values.positionRad.update(Units.rotationsToRadians(encoder.get()) + resetEncoderValueRad, timestampNow);
        values.velocityRadPerSec.update(Units.rotationsPerMinuteToRadiansPerSecond(encoder.getFrequency() * 60.0), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void setPosition(double positionRad) {
        resetEncoderValueRad = positionRad - Units.rotationsToRadians(encoder.get());
    }
}
