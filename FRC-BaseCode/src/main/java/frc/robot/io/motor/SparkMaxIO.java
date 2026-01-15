package frc.robot.io.motor;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;

public class SparkMaxIO implements MotorIO {

    private SparkMax sparkMaxMotor; // Motor controller instance
    private SparkClosedLoopController cl;
    private RelativeEncoder relEnc;

    // Constructor to initialize SparkMaxIO with the motor
    public SparkMaxIO(SparkMax motor) {
        this.sparkMaxMotor = motor;
        this.cl = this.sparkMaxMotor.getClosedLoopController();
        this.relEnc = this.sparkMaxMotor.getEncoder();
    }

    /**
     * Update the stored input values from hardware.
     * Fills the MotorIOValues struct with the latest readings (motor & encoder data).
     */
    @Override
    public void updateInputs(MotorIOValues inputs) {
        // Update motor-specific values (voltage, current, temperature) from the SparkMax
        double timestampNow = Timer.getFPGATimestamp();

        inputs.appliedVoltage.update(sparkMaxMotor.getAppliedOutput() * sparkMaxMotor.getBusVoltage(), timestampNow);
        inputs.currentAmps.update(sparkMaxMotor.getOutputCurrent(), timestampNow);
        inputs.tempCelsius.update(sparkMaxMotor.getMotorTemperature(), timestampNow);

        inputs.positionRad.update(relEnc.getPosition() * 2.0 * Math.PI, timestampNow);
        inputs.velocityRadPerSec.update(relEnc.getVelocity() * (2.0 * Math.PI / 60.0), timestampNow);
    }

    @Override
    public void setSpeed(double speed) {
        sparkMaxMotor.set(speed);  // Set the motor speed (closed-loop)
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



    // Closed loop sutff
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
