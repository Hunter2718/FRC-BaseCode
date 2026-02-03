package frc.robot.io.motor;

import java.util.ArrayList;
import java.util.List;

import frc.robot.io.motor.MotorIO.MotorIOValues;

public class MotorGroup {
    private List<MotorIO> motors;
    private int leaderIndex;
    private boolean leaderOnlyOutputs;
    private List<MotorIOValues> motorValues;

    public MotorGroup(List<MotorIO> motors) {
        this(motors, 0, false);
    }

    public MotorGroup(List<MotorIO> motors, int leaderIndex, boolean leaderOnlyOutputs) {
        this.motors = motors;
        this.leaderIndex = leaderIndex;
        this.leaderOnlyOutputs = leaderOnlyOutputs;

        this.motorValues = new ArrayList<>(this.motors.size());
    }

    private MotorIO leader() {
        return motors.get(leaderIndex);
    }

    public void updateInputs() {
        for (int i = 0; i < motors.size(); i++) {
            motors.get(i).updateInputs(motorValues.get(i));
        }
    }


    public void setVoltage(double volts) {
        if (leaderOnlyOutputs) leader().setVoltage(volts);
        else for (var m: motors) m.setVoltage(volts);
    }

    public void setVelocityRadPerSec(double radPerSec, double ffVolts) {
        if (leaderOnlyOutputs) leader().setVelocityRadPerSec(radPerSec, ffVolts);
        else for (var m : motors) m.setVelocityRadPerSec(radPerSec, ffVolts);
    }

    public void setPositionRad(double posRad, double ffVolts) {
        if (leaderOnlyOutputs) leader().setPositionRad(posRad, ffVolts);
        else for (var m : motors) m.setPositionRad(posRad, ffVolts);
    }

    public void stop() {
        if (leaderOnlyOutputs) leader().stop();
        else for (var m : motors) m.stop();
    }
    

    /**
     * Gets the ammount of motors in the group
     * @return the number of motors
     */
    public int getSize() {
        return motors.size();
    }

    public MotorIOValues getMotorValues(int index) {
        return motorValues.get(index);
    }
}
