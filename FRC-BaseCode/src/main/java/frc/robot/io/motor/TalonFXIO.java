package frc.robot.io.motor;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class TalonFXIO implements MotorIO {
    private TalonFX motor; // Motor controller instance

    // Constructor to initialize SparkMaxIO with the motor
    public TalonFXIO(TalonFX motor) {
        this.motor = motor;
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Update motor-specific values (voltage, current, temperature) from the SparkFlex
        double timestampNow = Timer.getFPGATimestamp();

        inputs.appliedVoltage.update(motor.getMotorVoltage().getValueAsDouble(), timestampNow);
        inputs.currentAmps.update(motor.getSupplyCurrent().getValueAsDouble(), timestampNow);
        inputs.tempCelsius.update(motor.getDeviceTemp().getValueAsDouble(), timestampNow);
    }

    /**
     * Run the motor with internal velocity control (closed-loop).
     * 
     * @param velocity - Desired velocity in RPM or other units
     */
    @Override
    public void setVelocity(double velocity) {
        motor.set(velocity);  // Set the motor speed (closed-loop)
    }

    /**
     * Run the motor with direct voltage (open-loop).
     * 
     * @param volts - Voltage to apply to the motor (open-loop)
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
