package frc.robot.subsystems.flyWheel;

public class FlyWheelSubsystemConstants {
    // If your sensor is on the motor but you want wheel rad/s, use motorRot / wheelRot.
    public static final double kIntakeGearRatio = 1.0;
    public static final double kShooterGearRatio = 1.0;

    // Preset setpoints (WHEEL rad/s). Adjust later.
    public static final double kIntakeWheelLowRadPerSec    = 250.0;
    public static final double kIntakeWheelNormalRadPerSec = 350.0;
    public static final double kIntakeWheelHighRadPerSec   = 450.0;

    public static final double kShooterWheelLowRadPerSec    = 250.0;
    public static final double kShooterWheelNormalRadPerSec = 350.0;
    public static final double kShooterWheelHighRadPerSec   = 450.0;

    // Convert wheel rad/s -> motor rad/s
    public static double wheelToMotorRadPerSec(double wheelRadPerSec, double gearRatio) {
        return wheelRadPerSec * gearRatio;
    }

    // Optional tolerance for "at speed" checks
    public static final double kShooterAtSpeedToleranceRadPerSec = 15.0;
    public static final double kIntakeAtSpeedToleranceRadPerSec = 15.0;

    // Optional arbitrary feedforward (volts). Start at 0 and add later if needed.
    public static final double kIntakeFFVolts = 0.0;
    public static final double kShooterFFVolts = 0.0;
}
