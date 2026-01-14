package frc.robot.io.motor;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class SparkIO implements MotorIO {
    private Spark motor; // Motor controller instance

    // Constructor to initialize SparkMaxIO with the motor
    public SparkIO(Spark motor) {
        this.motor = motor;
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Update motor-specific values (voltage, current, temperature) from the Spark
        double timestampNow = Timer.getFPGATimestamp();

        inputs.appliedVoltage.update(motor.getVoltage(), timestampNow);
    }

    /**
     * Run the motor with internal velocity control (closed-loop).
     * 
     * @param velocity - Desired velocity in RPM or other units like % speed for Spark, SparkMax, SparkFlex
     */
    @Override
    public void setVelocity(double velocity) {
        motor.set(velocity);  // Set the motor speed (closed-loop)
    }

    /**
     * Run the motor with direct voltage (open-loop).
     * 
     * @param volts - Voltage to apply to the motor (open-loop)
     * 
     * NOTE: This function *must* be called regularly in order for voltage compensation to work properly - unlike the ordinary set function, it is not "set it and forget it."
     */
    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);  // Apply the given voltage to the motor
    }

    /**
     * Stop output (safe neutral state) — set motor to neutral.
     */
    @Override
    public void stop() {
        motor.stopMotor();
    }
}
