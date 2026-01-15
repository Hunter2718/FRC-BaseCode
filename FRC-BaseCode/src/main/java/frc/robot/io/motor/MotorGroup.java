package frc.robot.io.motor;

import java.util.List;

import frc.robot.io.motor.MotorIO.MotorIOValues;

public class MotorGroup {
    private List<MotorIO> motors;
    private int leaderIndex;
    private boolean leaderOnlyOutputs;

    public MotorGroup(List<MotorIO> motors) {
        this(motors, 0, false);
    }

    public MotorGroup(List<MotorIO> motors, int leaderIndex, boolean leaderOnlyOutputs) {
        this.motors = motors;
        this.leaderIndex = leaderIndex;
        this.leaderOnlyOutputs = leaderOnlyOutputs;
    }

    private MotorIO leader() {
        return motors.get(leaderIndex);
    }

    public void updateInputs(List<MotorIOValues> motorValues) {
        int motorCount = motors.size();
        int motorValuesSize = motorValues.size();
    
        // Truncate the motorValues list if it's longer than the motors list
        if (motorValuesSize > motorCount) {
            motorValues = motorValues.subList(0, motorCount);
        }
        // Pad the motorValues list if it's shorter than the motors list
        else if (motorValuesSize < motorCount) {
            while (motorValues.size() < motorCount) {
                motorValues.add(new MotorIOValues());
            }
        }
    
        // Now update inputs based on the resized motorValues list
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
}
