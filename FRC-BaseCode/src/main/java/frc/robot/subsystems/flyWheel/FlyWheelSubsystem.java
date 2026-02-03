package frc.robot.subsystems.flyWheel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.encoder.EncoderIO;
import frc.robot.io.encoder.EncoderIO.EncoderIOValues;
import frc.robot.io.motor.MotorGroup;
import frc.robot.utils.UnitsUtils;

public class FlyWheelSubsystem extends SubsystemBase {
    private MotorGroup motors;
    private List<EncoderIO> encoders;

    private List<EncoderIOValues> encodersValues;

    private double targetWheelRadPerSec;

    private double gearRatio;
    private double ffVolts;
    private double atSpeedToleranceRadPerSec;

    public FlyWheelSubsystem(
        MotorGroup motors,
        double gearRatio,
        double ffVolts,
        double atSpeedToleranceRadPerSec
    ) {
        this(motors, null, gearRatio, ffVolts, atSpeedToleranceRadPerSec);
    }

    public FlyWheelSubsystem(
        MotorGroup motors,
        List<EncoderIO> encoders,
        double gearRatio,
        double ffVolts,
        double atSpeedToleranceRadPerSec
    ) {
        this.motors = motors;
        this.encoders = (encoders != null) ? encoders : Collections.emptyList();
        this.encodersValues = new ArrayList<>(this.encoders.size());
        this.gearRatio = gearRatio;
        this.ffVolts = ffVolts;
        this.atSpeedToleranceRadPerSec = atSpeedToleranceRadPerSec;

        targetWheelRadPerSec = 0.0;
    }

    @Override
    public void periodic() {
        motors.updateInputs();
    
        // Fix Size
        while (encodersValues.size() < encoders.size()) encodersValues.add(new EncoderIOValues());
        while (encodersValues.size() > encoders.size()) encodersValues.remove(encodersValues.size() - 1);

    
        // Now update inputs based on the resized motorValues list
        for (int i = 0; i < encoders.size(); i++) {
            encoders.get(i).updateInputs(encodersValues.get(i));
        }
    }

    public boolean hasEncoder() {
        return !encoders.isEmpty();
    }

      /** Motor rad/s (leader assumed index 0). */
    public double getMotorVelocityRadPerSec(int index) {
        return motors.getMotorValues(index).velocityRadPerSec.value;
    }

    /** Motor rad (leader assumed index 0). */
    public double getMotorPositionRad(int index) {
        return motors.getMotorValues(index).positionRad.value;
    }

    /** Encoder count (0 if none). */
    public int getEncoderCount() {
        return encoders.size();
    }

    /** Raw encoder rad/s (whatever shaft the encoder is on). */
    public double getEncoderVelocityRadPerSec(int index) {
        return encodersValues.get(index).velocityRadPerSec.value;
    }

    /** Raw encoder rad (whatever shaft the encoder is on). */
    public double getEncoderPositionRad(int index) {
        return encodersValues.get(index).positionRad.value;
    }

    public double getWheelVelocityRadPerSec(int motorIndex) {
        // wheel encoder, you can do:
        // if (getEncoderCount() > 0) return getEncoderVelocityRadPerSec(0);

        return getMotorVelocityRadPerSec(motorIndex) / gearRatio;
    }

    public double getTargetWheelRadPerSec() {
        return targetWheelRadPerSec;
    }

    /** Closed-loop speed command (wheel rad/s). */
    public void setTargetWheelRadPerSec(double wheelRadPerSec) {
        targetWheelRadPerSec = wheelRadPerSec;

        double motorRadPerSec = UnitsUtils.wheelToMotorRadPerSec(wheelRadPerSec, gearRatio);
        motors.setVelocityRadPerSec(motorRadPerSec, ffVolts);
    }

    public boolean atSpeed(int motorIndex) {
        return Math.abs(getWheelVelocityRadPerSec(motorIndex) - targetWheelRadPerSec)
            <= atSpeedToleranceRadPerSec;
    }

    public void stop() {
        targetWheelRadPerSec = 0.0;
        motors.stop();
    }
}
