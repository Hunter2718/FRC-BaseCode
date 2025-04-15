package frc.robot.IO.Gryo;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Utils.TimestampedValue;

public interface GryoIO { 
    public class GryoIOValues {

        public TimestampedValue<Rotation3d> position = 
            new TimestampedValue<>(
                new Rotation3d(0, 0, 0),
                0
            );
        
        public List<TimestampedValue<Rotation3d>> pastOdometryPositions = 
            Arrays.asList(
                new TimestampedValue<>(new Rotation3d(0.0, 0.0, 0.0), 0)
            );


        public TimestampedValue<Double> velocityYaw = new TimestampedValue<>(0.0, 0); //deg/s
        public TimestampedValue<Double> velocityPitch = new TimestampedValue<>(0.0, 0); //deg/s
        public TimestampedValue<Double> velocityRoll = new TimestampedValue<>(0.0, 0); //deg/s
    }

    // Update the gryo values — read from hardware and update the provided values
    default void updateInputs(GryoIOValues values) {}

    // Zero Yaw to a certin value
    default void zeroYawPosition(double position) {}

    // Zero Pitch to a certin value
    default void zeroPitchPosition(double position) {}

    // Zero Roll to a certin value
    default void zeroRollPosition(double position) {}

}