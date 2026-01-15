package frc.robot.io.motor;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Timer;

public class TalonFXIO implements MotorIO {
    private TalonFX motor; // Motor controller instance
    private VelocityVoltage velReq;
    private PositionVoltage posReq;

    // Constructor to initialize SparkMaxIO with the motor
    public TalonFXIO(TalonFX motor) {
        this.motor = motor;
        this.velReq = new VelocityVoltage(0.0);
        this.posReq = new PositionVoltage(0.0);
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

        inputs.positionRad.update(motor.getPosition().getValueAsDouble() * 2.0 * Math.PI, timestampNow);
        inputs.velocityRadPerSec.update(motor.getVelocity().getValueAsDouble() * 2.0 * Math.PI, timestampNow);
    }

    @Override
    public void setSpeed(double speed) {
        motor.set(speed);  // Set the motor speed (closed-loop)
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



    // Closed loop stuff
    @Override
    public boolean supportsVelocityClosedLoop() {
        return true;
    }

    @Override
    public boolean supportsPositionClosedLoop() {
        return true;
    }

    @Override
    public void setVelocityRadPerSec(double motorRadPerSec, double ffVolts) {
        double rps = motorRadPerSec / (2.0 * Math.PI);
        motor.setControl(velReq.withVelocity(rps).withFeedForward(ffVolts));
    }

    @Override
    public void setPositionRad(double motorPosRad, double ffVolts) {
        double rotations = motorPosRad / (2.0 * Math.PI);
        motor.setControl(posReq.withPosition(rotations).withFeedForward(ffVolts));
    }
}
