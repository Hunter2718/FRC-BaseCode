package frc.robot.IO.Encoder;

import com.ctre.phoenix6.Utils;
import com.revrobotics.AbsoluteEncoder;

public class AbsoluteEncoderIO implements EncoderIO {
    private AbsoluteEncoder encoder;
    private double resetEncoderValue;

    /**
     * 
     * @param encoder Pre-Configured absoluet encoder
     */
    public AbsoluteEncoderIO(AbsoluteEncoder encoder) {
        this.encoder = encoder;
        resetEncoderValue = 0.0;
    }
    

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());
        values.position.update(encoder.getPosition() - resetEncoderValue, timestampNow);
        values.velocity.update(encoder.getVelocity(), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void zeroPosition(double position) {
        resetEncoderValue = encoder.getPosition();
    }
}
