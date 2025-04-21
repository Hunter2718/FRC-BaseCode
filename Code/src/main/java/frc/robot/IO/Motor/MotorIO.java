package frc.robot.IO.Motor;

import frc.robot.Utils.TimestampedValue;

/**
 * MotorIO abstracts a pre-configured motor.
 * It separates runtime control from configuration.
 * Supports Smart Motion (position control), velocity control, and voltage control.
 */
public interface MotorIO {

    /** Container for all motor inputs & sensor values. */
    public class MotorIOValues {
        public MotorIOValues() {}

        public TimestampedValue<Double> appliedVoltage = new TimestampedValue<>(0.0, 0); // volts
        public TimestampedValue<Double> currentAmps = new TimestampedValue<>(0.0, 0);    // amps
        public TimestampedValue<Double> tempCelsius = new TimestampedValue<>(0.0, 0);    // degrees Celsius
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
