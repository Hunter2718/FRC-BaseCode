package frc.robot.io.motor;

import java.util.List;

import frc.robot.io.motor.MotorIO.MotorIOValues;

public class MotorGroup {
    private List<MotorIO> motors;

    public MotorGroup(List<MotorIO> motors) {
        this.motors = motors;
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
    

    public void setSingularMotorVelocity(double velocity, int index) {
        motors.get(index).setVelocity(velocity);
    }

    public void setSingularMotorVoltage(double volts, int index) {
        motors.get(index).setVoltage(volts);
    }

    public void stopSingularMotor(int index) {
        motors.get(index).stop();
    }

    public void setAllVelocity(double velocity) {
        for (MotorIO motor : motors) {
            motor.setVelocity(velocity);
        }
    }

    public void setAllVoltage(double volts) {
        for (MotorIO motor : motors) {
            motor.setVoltage(volts);
        }
    }

    public void stopAll() {
        for (MotorIO motor : motors) {
            motor.stop();
        }
    }

    // Synchronize motor velocities to ensure they all start/stop at the same time
    /**
     * 
     * @param velocityToSync sync all motors to the same velocity
     * Most commonly will run like
     * synchronizeMotors(motorValues.get(index).encoderValues.velocity)
     * This will sync all motors to the velocity of the motor from the motor values. If
     * you don't do this then it is basically the same as setAllVelocity(velocityToSync)
     * 
     * This does the same as setAllVelocity(velocity)
     */
    public void synchronizeMotors(double velocityToSync) {
        for (MotorIO motor : motors) {
            motor.setVelocity(velocityToSync);
        }
    }
}
