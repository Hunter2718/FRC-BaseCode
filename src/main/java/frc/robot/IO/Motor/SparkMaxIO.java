package frc.robot.IO.Motor;

import java.util.List;

import com.revrobotics.spark.SparkMax;

import frc.robot.IO.Encoder.EncoderIO;
import frc.robot.IO.Encoder.EncoderIO.EncoderIOValues;

public class SparkMaxIO implements MotorIO {

    private final SparkMax sparkMaxMotor; // Motor controller instance
    private final List<EncoderIO> encoders;  // List of encoders attached to this motor

    // Constructor to initialize SparkMaxIO with the motor and encoders
    public SparkMaxIO(SparkMax motor, List<EncoderIO> encoders) {
        this.sparkMaxMotor = motor;
        this.encoders = encoders;
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Clear the old encoder values to prevent stale data
        inputs.encoderValues.clear();

        // Update encoder values for each encoder connected to the motor
        for (EncoderIO encoder : encoders) {
            if (encoder != null) {
                EncoderIOValues encoderInputs = new EncoderIOValues();
                encoder.updateInputs(encoderInputs);  // Update encoder data from hardware
                inputs.encoderValues.add(encoderInputs.clone());
            }
        }

        // Update motor-specific values (voltage, current, temperature) from the SparkMax
        inputs.appliedVoltage = sparkMaxMotor.getAppliedOutput() * sparkMaxMotor.getBusVoltage();
        inputs.currentAmps = sparkMaxMotor.getOutputCurrent();
        inputs.tempCelsius = sparkMaxMotor.getMotorTemperature();
    }

    /**
     * Run the motor with internal velocity control (closed-loop).
     * 
     * @param velocity - Desired velocity in RPM or other units
     */
    @Override
    public void setVelocity(double velocity) {
        sparkMaxMotor.set(velocity);  // Set the motor speed (closed-loop)
    }

    /**
     * Run the motor with direct voltage (open-loop).
     * 
     * @param volts - Voltage to apply to the motor (open-loop)
     */
    @Override
    public void setVoltage(double volts) {
        sparkMaxMotor.setVoltage(volts);  // Apply the given voltage to the motor
    }

    /**
     * Stop output (safe neutral state) — set motor to neutral.
     */
    @Override
    public void stop() {
        sparkMaxMotor.set(0);  // Stop the motor (neutral state)
    }
}
