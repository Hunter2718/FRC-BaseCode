package frc.robot.IO.Motor;

import java.util.ArrayList;
import java.util.List;

import frc.robot.IO.Encoder.EncoderIO;

/**
 * MotorIO abstracts a pre-configured motor.
 * It separates runtime control from configuration.
 * Supports Smart Motion (position control), velocity control, and voltage control.
 */
public interface MotorIO {

    /** Container for all motor inputs & sensor values. */
    public class MotorIOValues {
        public double appliedVoltage = 0.0; // volts
        public double currentAmps = 0.0;    // amps
        public double tempCelsius = 0.0;    // degrees Celsius

        List<EncoderIO.EncoderIOValues> encoderValues = new ArrayList<>();
    }

    /**
     * Update the stored input values from hardware.
     * Should fill the MotorIOValues struct.
     */
    default void updateInputs(MotorIOValues inputs) {}

    /**
     * Run the motor with internal velocity control (closed-loop).
     * Units: meters per second (m/s) or radians per second (rad/s).
     */
    default void setVelocity(double velocity) {}

    /**
     * Run the motor with direct voltage (open-loop).
     * Units: volts.
     */
    default void setVoltage(double volts) {}

    /**
     * Stop output (safe neutral state).
     */
    default void stop() {}
}
