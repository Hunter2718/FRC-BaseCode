package frc.robot.IO.Encoder;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class DutyCycleEncoderIO implements EncoderIO{
    private DutyCycleEncoder encoder;
    private double resetEncoderValue;

    /**
     * 
     * @param encoder Pre-Configured absoluet encoder
     */
    public DutyCycleEncoderIO(DutyCycleEncoder encoder) {
        this.encoder = encoder;
        resetEncoderValue = 0.0;
    }
    

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());
        values.position.update(encoder.get() - resetEncoderValue, timestampNow);
        values.velocity.update(encoder.getFrequency() * 60.0, timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void zeroPosition(double position) {
        resetEncoderValue = encoder.get();
    }
}
