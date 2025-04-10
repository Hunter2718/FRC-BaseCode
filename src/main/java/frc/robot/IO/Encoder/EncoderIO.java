package frc.robot.IO.Encoder;

public interface EncoderIO {

    public class EncoderIOValues {
        public double position = 0.0; // In rotations
        public double velocity = 0.0; // In rotations/min
        public double[] timestamps = new double[] {};
        public double[] positions = new double[] {};

        // Constructor for deep copying
        public EncoderIOValues() {}

        // Copy constructor for deep copying
        public EncoderIOValues(EncoderIOValues other) {
            this.position = other.position;
            this.velocity = other.velocity;
        }

        // Implement a clone method
        public EncoderIOValues clone() {
            return new EncoderIOValues(this);
        }
    }

    // Update the encoder values — read from hardware and update the provided values
    default void updateInputs(EncoderIOValues values) {}

    // Optional: Reset encoder position
    default void zeroPosition(double position) {}
}
