package frc.robot.IO.Gryo;

import edu.wpi.first.math.geometry.Rotation2d;

public interface GryoIO { 
    public class GryoIOValues {
        public boolean connected = false;

        public Rotation2d positionYaw = new Rotation2d();
        public double velocityRadPerSecYaw = 0.0;
        public double[] odometryTimestampsYaw = new double[] {};
        public Rotation2d[] odometryPositionsYaw = new Rotation2d[] {};

        public Rotation2d positionPitch = new Rotation2d();
        public double velocityRadPerSecPitch = 0.0;
        public double[] odometryTimestampsPitch = new double[] {};
        public Rotation2d[] odometryPositionsPitch = new Rotation2d[] {};
    
        public Rotation2d positionRoll = new Rotation2d();
        public double velocityRadPerSecRoll = 0.0;
        public double[] odometryTimestampsRoll = new double[] {};
        public Rotation2d[] odometryPositionsRoll = new Rotation2d[] {};
        
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