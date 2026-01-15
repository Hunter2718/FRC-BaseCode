package frc.robot.io.motor;

import frc.robot.utils.TimestampedValue;

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

        public TimestampedValue<Double> positionRad = new TimestampedValue<Double>(0.0, 0);
        public TimestampedValue<Double> velocityRadPerSec = new TimestampedValue<Double>(0.0, 0);
    }

    /**
     * Update the stored input values from hardware.
     * Should fill the MotorIOValues struct.
     */
    default void updateInputs(MotorIOValues inputs) {}

    default void setSpeed(double speed) {}

    /**
     * Run the motor with direct voltage (open-loop).
     * Units: volts.
     */
    default void setVoltage(double volts) {}

    /**
     * Stop output (safe neutral state).
     */
    default void stop() {}

    // Capability flags (lets you fall back safely)
  default boolean supportsVelocityClosedLoop() { return false; }
  default boolean supportsPositionClosedLoop() { return false; }

  // Closed-loop commands (motor-shaft units)
  default void setVelocityRadPerSec(double motorRadPerSec, double ffVolts) {}
  default void setPositionRad(double motorPosRad, double ffVolts) {}
}
