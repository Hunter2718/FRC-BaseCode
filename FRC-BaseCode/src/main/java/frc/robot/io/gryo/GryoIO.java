package frc.robot.io.gryo;

import java.util.ArrayDeque;
import java.util.Deque;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.utils.TimestampedValue;

public interface GryoIO { 
    public class GryoIOValues {

        public TimestampedValue<Rotation3d> position = 
            new TimestampedValue<>(
                new Rotation3d(0, 0, 0),
                0
            );
        
        public static final double kPastOdomWindowSec = 2.0;
        public Deque<TimestampedValue<Rotation3d>> pastOdometryPositions = new ArrayDeque<>();


        public TimestampedValue<Double> velocityYawRadPerSec = new TimestampedValue<>(0.0, 0); //radians/second
        public TimestampedValue<Double> velocityPitchRadPerSec = new TimestampedValue<>(0.0, 0); //radians/second
        public TimestampedValue<Double> velocityRollRadPerSec = new TimestampedValue<>(0.0, 0); //radians/second
    }

    // Update the gryo values — read from hardware and update the provided values
    default void updateInputs(GryoIOValues values) {}

    // set Yaw to a certin value
    default void setYawPosition(double positionRad) {}

    // set Pitch to a certin value
    default void setPitchPosition(double positionRad) {}

    // set Roll to a certin value
    default void setRollPosition(double positionRad) {}

}