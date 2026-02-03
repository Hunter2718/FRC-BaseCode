package frc.robot.subsystems.positionPiece;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.encoder.EncoderIO;
import frc.robot.io.encoder.EncoderIO.EncoderIOValues;
import frc.robot.io.motor.MotorGroup;
import frc.robot.utils.UnitsUtils;

public class PositionPieceSubsystem extends SubsystemBase {
    private MotorGroup motors;
    private List<EncoderIO> encoders;
    private List<EncoderIOValues> encodersValues;

    private double gearRatio;

    private double goalPieceRad;
    private double manualMaxVolts;
    private double minPieceRad;
    private double maxPieceRad;
    private double ffVolts;
    private double atGoalToleranceRad;

    public PositionPieceSubsystem(
        MotorGroup motors,
        double gearRatio,
        double manualMaxVolts,
        double minPieceRad,
        double maxPieceRad,
        double ffVolts,
        double atGoalToleranceRad
    ) {
        this(motors, null, gearRatio, manualMaxVolts, minPieceRad, maxPieceRad, ffVolts, atGoalToleranceRad);
    }

    public PositionPieceSubsystem(
        MotorGroup motors,
        List<EncoderIO> encoders,
        double gearRatio,
        double manualMaxVolts,
        double minPieceRad,
        double maxPieceRad,
        double ffVolts,
        double atGoalToleranceRad
    ) {
        this.motors = motors;
        this.encoders = (encoders != null) ? encoders : Collections.emptyList();
        this.encodersValues = new ArrayList<>(this.encoders.size());

        this.gearRatio = gearRatio;
        this.manualMaxVolts = manualMaxVolts;
        this.minPieceRad = minPieceRad;
        this.maxPieceRad = maxPieceRad;
        this.ffVolts = ffVolts;
        this.atGoalToleranceRad = atGoalToleranceRad;
        
        goalPieceRad = 0.0;
    }

    @Override
    public void periodic() {
        motors.updateInputs();
    
        // Fix size
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

    public double getMotorPositionRad(int index) {
        return motors.getMotorValues(index).positionRad.value;
    }

    public double getMotorVelocityRadPerSec(int index) {
        return motors.getMotorValues(index).velocityRadPerSec.value;
    }

    public double getEncoderPositionRad(int index) {
        return encodersValues.get(index).positionRad.value;
    }

    public double getEncoderVelocityRadPerSec(int index) {
        return encodersValues.get(index).velocityRadPerSec.value;
    }

    public double getGoalPieceRad() {
        return goalPieceRad;
    }

    public void setManualVolts(double volts) {
        volts = MathUtil.clamp(volts, -manualMaxVolts, manualMaxVolts);
        motors.setVoltage(volts);
    }

    public void moveToPositionRad(double endPieceRad, double currentPieceRad, double currentMotorRad) {
        goalPieceRad = MathUtil.clamp(endPieceRad, minPieceRad, maxPieceRad);

        double motorGoalRad = UnitsUtils.pieceRadToMotorRad(goalPieceRad, currentPieceRad, currentMotorRad, gearRatio);
        motors.setPositionRad(motorGoalRad, ffVolts);
    }

    public boolean atGoal(int encoderIndex) {
        return Math.abs(getEncoderPositionRad(encoderIndex) - goalPieceRad) <= atGoalToleranceRad;
    }

    public int getEncoderCount() { return encoders.size(); }

    public int getMotorCount() { return motors.getSize(); }


    public void stop() {
        motors.stop();
    }
}
