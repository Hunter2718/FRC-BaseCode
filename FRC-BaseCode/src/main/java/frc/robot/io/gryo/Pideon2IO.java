package frc.robot.io.gryo;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.TimestampedValue;

public class Pideon2IO implements GryoIO {
    private Pigeon2 pigeon;
    /**
     * 
     * @param pigeon Pre-configured pigeon
     */
    public Pideon2IO(Pigeon2 pigeon) {
        this.pigeon = pigeon;
    }
    

    @Override
    public void updateInputs(GryoIOValues values) {
        double timestampNow = Timer.getFPGATimestamp();
        values.position.update(pigeon.getRotation3d(), timestampNow);

        values.pastOdometryPositions.add(
            new TimestampedValue<Rotation3d>(pigeon.getRotation3d(), timestampNow)
        );

        values.velocityYawRadPerSec.update(Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble()), timestampNow);
        //TODO: make sure these values are correct possible need to switch X and Y (pitch and roll)
        values.velocityPitchRadPerSec.update(Units.degreesToRadians(pigeon.getAngularVelocityXWorld().getValueAsDouble()), timestampNow);
        values.velocityRollRadPerSec.update(Units.degreesToRadians(pigeon.getAngularVelocityYWorld().getValueAsDouble()), timestampNow);
    }

    @Override
    /**
     * @param positionRad Position to set Yaw to is in Radians
     */
    public void setYawPosition(double positionRad) {
        pigeon.setYaw(Units.radiansToDegrees(positionRad));
    }
}
