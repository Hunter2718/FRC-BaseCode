package frc.robot.IO.Motor;

import com.ctre.phoenix6.Utils;
import com.revrobotics.spark.SparkMax;

public class SparkMaxIO implements MotorIO {

    private SparkMax sparkMaxMotor; // Motor controller instance

    // Constructor to initialize SparkMaxIO with the motor
    public SparkMaxIO(SparkMax motor) {
        this.sparkMaxMotor = motor;
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Update motor-specific values (voltage, current, temperature) from the SparkMax
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());

        inputs.appliedVoltage.update(sparkMaxMotor.getAppliedOutput() * sparkMaxMotor.getBusVoltage(), timestampNow);
        inputs.currentAmps.update(sparkMaxMotor.getOutputCurrent(), timestampNow);
        inputs.tempCelsius.update(sparkMaxMotor.getMotorTemperature(), timestampNow);
    }

    /**
     * Run the motor with internal velocity control (closed-loop).
     * 
     * @param velocity - Desired velocity in RPM or other units like % speed for Spark, SparkMax, SparkFlex
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
        sparkMaxMotor.stopMotor();
    }
}
