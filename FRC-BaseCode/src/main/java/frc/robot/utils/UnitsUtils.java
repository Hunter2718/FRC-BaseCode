package frc.robot.utils;

public class UnitsUtils {
    // Convert wheel rad/s -> motor rad/s
    public static double wheelToMotorRadPerSec(double wheelRadPerSec, double gearRatio) {
        return applyGearRatio(wheelRadPerSec, gearRatio);
    }

    // Piece rad -> motor rad
    public static double pieceRadToMotorRad(double goalPieceRad, double currentPieceRad, double currentMotorRad, double gearRatio) {
        double error = goalPieceRad - currentPieceRad;
        error = Math.atan2(Math.sin(error), Math.cos(error));
        return (currentMotorRad + applyGearRatio(error, gearRatio));
    }

    /**
     * 
     * @param value the value to change based on gear ratio
     * @param gearRatio the gear ratio from source to destination (motor -> piece)
     * @return the updated value
     */
    public static double applyGearRatio(double value, double gearRatio) {
        return value * gearRatio;
    }
}
