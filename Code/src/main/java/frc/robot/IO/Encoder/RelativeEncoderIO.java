package frc.robot.IO.Encoder;

import com.revrobotics.RelativeEncoder;

import com.ctre.phoenix6.Utils;

public class RelativeEncoderIO implements EncoderIO {
    private RelativeEncoder encoder;
    
    public RelativeEncoderIO(RelativeEncoder encoder) {
        this.encoder = encoder;
    }

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());
        values.position.update(encoder.getPosition(), timestampNow);
        values.velocity.update(encoder.getVelocity(), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void zeroPosition(double position) {
        encoder.setPosition(position);
    }
}
