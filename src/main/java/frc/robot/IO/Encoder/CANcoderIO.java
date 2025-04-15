package frc.robot.IO.Encoder;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;

public class CANcoderIO implements EncoderIO {
    private CANcoder encoder;

    public CANcoderIO(CANcoder encoder) {
        this.encoder = encoder;
    }

    // Update the encoder values — read from hardware and update the provided values
    @Override
    public void updateInputs(EncoderIOValues values) {
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());
        values.position.update(encoder.getPosition().getValueAsDouble(), timestampNow);
        values.velocity.update(encoder.getVelocity().getValueAsDouble(), timestampNow);
    }

    // Optional: Reset encoder position
    @Override
    public void zeroPosition(double position) {
        encoder.setPosition(position);
    }
}
