package frc.robot.IO.Encoder;

import com.revrobotics.RelativeEncoder;

public class RelativeEncoderIO implements EncoderIO {
    RelativeEncoder encoder;
    
    public RelativeEncoderIO(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        values.position = encoder.getPosition();
        values.velocity = encoder.getVelocity();
    }

    // Optional: Reset encoder position
    @Override
    public void zeroPosition(double position) {
        encoder.setPosition(position);
    }
}
