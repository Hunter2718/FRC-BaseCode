package frc.robot.IO.Encoder;

import frc.robot.Utils.TimestampedValue;

public interface EncoderIO {

    public class EncoderIOValues {
        public TimestampedValue<Double> position = new TimestampedValue<>(null, 0.0); // rotations
        public TimestampedValue<Double> velocity = new TimestampedValue<>(null, 0.0); // rotations/min (RPM)
    }

    // Update the encoder values — read from hardware and update the provided values
    default void updateInputs(EncoderIOValues values) {}

    // Optional: Reset encoder position
    default void zeroPosition(double position) {}
}
