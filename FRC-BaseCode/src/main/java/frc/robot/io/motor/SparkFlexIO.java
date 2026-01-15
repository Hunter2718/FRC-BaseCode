package frc.robot.io.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.wpilibj.Timer;

public class SparkFlexIO implements MotorIO {
    private SparkFlex motor; // Motor controller instance
    private SparkClosedLoopController cl;
    private RelativeEncoder relEnc;

    // Constructor to initialize SparkMaxIO with the motor
    public SparkFlexIO(SparkFlex motor) {
        this.motor = motor;
        this.cl = this.motor.getClosedLoopController();
        this.relEnc = this.motor.getEncoder();
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Update motor-specific values (voltage, current, temperature) from the SparkFlex
        double timestampNow = Timer.getFPGATimestamp();

        inputs.appliedVoltage.update(motor.getAppliedOutput() * motor.getBusVoltage(), timestampNow);
        inputs.currentAmps.update(motor.getOutputCurrent(), timestampNow);
        inputs.tempCelsius.update(motor.getMotorTemperature(), timestampNow);

        inputs.positionRad.update(relEnc.getPosition() * 2.0 * Math.PI, timestampNow);
        inputs.velocityRadPerSec.update(relEnc.getVelocity() * (2.0 * Math.PI / 60.0), timestampNow);
    }


    @Override
    public void setSpeed(double speed) {
        motor.set(speed);
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
        motor.stopMotor();  // Stop the motor (neutral state)
    }


    @Override
    public boolean supportsPositionClosedLoop() {
        return true;
    }

    @Override
    public boolean supportsVelocityClosedLoop() {
        return true;
    }

    @Override
    public void setVelocityRadPerSec(double motorRadPerSec, double ffVolts) {
        double rpm = motorRadPerSec * 60 / (2.0 * Math.PI);
        cl.setSetpoint(
            rpm,
            SparkBase.ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }

    @Override
    public void setPositionRad(double motorPosRad, double ffVolts) {
        double rotations = motorPosRad / (2.0 * Math.PI);
        cl.setSetpoint(
            rotations,
            SparkBase.ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            ffVolts,
            SparkClosedLoopController.ArbFFUnits.kVoltage
        );
    }
}
