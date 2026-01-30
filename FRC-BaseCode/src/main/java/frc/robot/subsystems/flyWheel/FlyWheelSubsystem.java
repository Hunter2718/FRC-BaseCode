package frc.robot.subsystems.flyWheel;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.encoder.EncoderIO;
import frc.robot.io.encoder.EncoderIO.EncoderIOValues;
import frc.robot.io.motor.MotorGroup;
import frc.robot.io.motor.MotorIO.MotorIOValues;

public class FlyWheelSubsystem extends SubsystemBase {
    private MotorGroup motors;
    private List<EncoderIO> encoders;

    private List<MotorIOValues> motorsValues;
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
        this.motorsValues = new ArrayList<>(motors.getSize());
        this.encodersValues = new ArrayList<>(encoders.size());
        this.gearRatio = gearRatio;
        this.ffVolts = ffVolts;
        this.atSpeedToleranceRadPerSec = atSpeedToleranceRadPerSec;

        targetWheelRadPerSec = 0.0;
    }

    @Override
    public void periodic() {
        motors.updateInputs(motorsValues);
        
        int encoderCount = encoders.size();
        int encodersValuesSize = encodersValues.size();
    
        // Truncate the motorValues list if it's longer than the motors list
        if (encodersValuesSize > encoderCount) {
            encodersValues = encodersValues.subList(0, encoderCount);
        }
        // Pad the motorValues list if it's shorter than the motors list
        else if (encodersValuesSize < encoderCount) {
            while (encodersValues.size() < encoderCount) {
                encodersValues.add(new EncoderIOValues());
            }
        }
    
        // Now update inputs based on the resized motorValues list
        for (int i = 0; i < encoders.size(); i++) {
            encoders.get(i).updateInputs(encodersValues.get(i));
        }
    }

    public boolean hasEncoder() {
        return !encoders.isEmpty();
    }

      /** Motor rad/s (leader assumed index 0). */
    public double getMotorVelocityRadPerSec() {
        return motorsValues.get(0).velocityRadPerSec.value;
    }

    /** Motor rad (leader assumed index 0). */
    public double getMotorPositionRad() {
        return motorsValues.get(0).positionRad.value;
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

    public double getWheelVelocityRadPerSec() {
        // wheel encoder, you can do:
        // if (getEncoderCount() > 0) return getEncoderVelocityRadPerSec(0);

        return getMotorVelocityRadPerSec() / gearRatio;
    }

    public double getTargetWheelRadPerSec() {
        return targetWheelRadPerSec;
    }

    /** Closed-loop speed command (wheel rad/s). */
    public void setTargetWheelRadPerSec(double wheelRadPerSec) {
        targetWheelRadPerSec = wheelRadPerSec;

        double motorRadPerSec = FlyWheelSubsystemConstants.wheelToMotorRadPerSec(wheelRadPerSec, gearRatio);
        motors.setVelocityRadPerSec(motorRadPerSec, ffVolts);
    }

    public boolean atSpeed() {
        return Math.abs(getWheelVelocityRadPerSec() - targetWheelRadPerSec)
            <= atSpeedToleranceRadPerSec;
    }

    public void stop() {
        targetWheelRadPerSec = 0.0;
        motors.stop();
    }
}
