package frc.robot.IO.Gryo;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import frc.robot.Utils.TimestampedValue;

public class Pideon2IO implements GryoIO {
    private Pigeon2 pigeon;
    /**
     * 
     * @param pigeon Pre-configured pigeon
     */
    public Pideon2IO (Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }
    

    @Override
    public void updateInputs(GryoIOValues values) {
        double timestampNow = Utils.fpgaToCurrentTime(Utils.getCurrentTimeSeconds());
        values.position.update(pigeon.getRotation3d(), timestampNow);

        values.pastOdometryPositions.add(
            new TimestampedValue<Rotation3d>(pigeon.getRotation3d(), timestampNow)
        );

        values.velocityYaw.update(pigeon.getAngularVelocityZWorld().getValueAsDouble(), timestampNow);
        //TODO: make sure these values are correct possible need to switch X and Y (pitch and roll)
        values.velocityPitch.update(pigeon.getAngularVelocityXWorld().getValueAsDouble(), timestampNow);
        values.velocityRoll.update(pigeon.getAngularVelocityYWorld().getValueAsDouble(), timestampNow);
    }

    @Override
    /**
     * @param positionDeg Position to set Yaw to is in Degrees
     */
    public void zeroYawPosition(double positionDeg) {
        pigeon.setYaw(positionDeg);
    }
}
