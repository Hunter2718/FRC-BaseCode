package frc.robot.io.encoder;

import frc.robot.utils.TimestampedValue;

public interface EncoderIO {

    public class EncoderIOValues {
        public TimestampedValue<Double> positionRad = new TimestampedValue<>(0.0, 0.0); // radians
        public TimestampedValue<Double> velocityRadPerSec = new TimestampedValue<>(0.0, 0.0); // radians/second
    }

    // Update the encoder values — read from hardware and update the provided values
    default void updateInputs(EncoderIOValues values) {}

    // Optional: set encoder position
    default void setPosition(double positionRad) {}
}
