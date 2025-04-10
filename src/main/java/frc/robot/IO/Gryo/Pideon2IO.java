package frc.robot.IO.Gryo;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

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
    //TODO: FINISH THIS
    public void updateInputs(GryoIOValues values) {
        values.connected = pigeon.isConnected();

        values.positionYaw = Rotation2d.fromDegrees(pigeon.getYaw().getValueAsDouble());
        values.velocityRadPerSecYaw = Units.degreesToRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
    }

    @Override
    /**
     * @param positionDeg Position to set Yaw to is in Degrees
     */
    public void zeroYawPosition(double positionDeg) {
        pigeon.setYaw(positionDeg);
    }
}
