package frc.robot.subsystems.positionPiece;

public class PositionPieceSubsystemConstants {
  // If you command motor position loops, you need joint->motor mapping.
  // motor Rad per piece Rad
  public static final double kTurretGearRatio = 100.0; // placeholder
  public static final double kIntakePivotJointGearRatio = 100.0; // placeholder

  // Soft limits (joint angle radians) - placeholders
  public static final double kMinTurretRad = -0.2;
  public static final double kMaxTurretRad =  2.6;
  public static final double kMinIntakePivotJointRad = -0.2;
  public static final double kMaxIntakePivotJointRad =  2.6;

  // Teleop
  public static final double kTurretManualMaxVolts = 6.0;
  public static final double kTurretDeadband = 0.08;
  public static final double kIntakePivotJointManualMaxVolts = 6.0;
  public static final double kIntakePivotJointDeadband = 0.08;

  // Closed-loop tolerance (joint radians)
  public static final double kTurretAtGoalToleranceRad = 0.02;
  public static final double kIntakePivotJointAtGoalToleranceRad = 0.02;

  // Optional feedforward volts for gravity later (start 0)
  public static final double kTurretFFVolts = 0.0;
  public static final double kIntakePivotJointFFVolts = 0.0;

  // Presets (joint radians) - placeholders
  public static final double kTurretPosOneRad = 0.0;
  public static final double kTurretPosTwoRad = 0.6;
  public static final double kTurretPosThreeRad = 1.8;
  public static final double kIntakePivotJointPosOneRad = 0.0;
  public static final double kIntakePivotJointPosTwoRad = 0.6;
  public static final double kIntakePivotJointPosThreeRad = 1.8;

  public static double pieceRadToMotorRad(double goalPieceRad, double currentPieceRad, double currentMotorRad, double gearRatio) {
    double error = goalPieceRad - currentPieceRad;
    error = Math.atan2(Math.sin(error), Math.cos(error));
    return (currentMotorRad + error * gearRatio);
  }
}
