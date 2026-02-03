package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class GeometryUtil {
  private GeometryUtil() {}

  /** Returns the angle (field frame) from 'from' to 'to'. */
  public static Rotation2d angleTo(Translation2d from, Translation2d to) {
    return to.minus(from).getAngle();
  }

  /** Unit vector pointing from 'from' to 'to'. Returns (0,0) if zero length. */
  public static Translation2d unitTo(Translation2d from, Translation2d to) {
    Translation2d d = to.minus(from);
    double n = d.getNorm();
    return (n < 1e-9) ? new Translation2d() : d.div(n);
  }

  /** Dot product of two 2D vectors. */
  public static double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  /** Projection of v onto unit direction u (u should be unit length). */
  public static double along(Translation2d v, Translation2d uUnit) {
    return dot(v, uUnit);
  }

  /** Perpendicular component magnitude (sideways) relative to u (u should be unit). */
  public static double sideways(Translation2d v, Translation2d uUnit) {
    // v_perp = v - u*(v·u)
    Translation2d perp = v.minus(uUnit.times(dot(v, uUnit)));
    return perp.getNorm();
  }

  /** Rotate a field-frame angle into robot frame using robot yaw (field->robot). */
  public static Rotation2d fieldAngleToRobot(Rotation2d fieldAngle, Rotation2d robotYaw) {
    return fieldAngle.minus(robotYaw);
  }

  /** Rotate a robot-frame angle into field frame using robot yaw (robot->field). */
  public static Rotation2d robotAngleToField(Rotation2d robotAngle, Rotation2d robotYaw) {
    return robotAngle.plus(robotYaw);
  }

  /** Vector from robot pose to a field target. */
  public static Translation2d toTarget(Pose2d robotPoseField, Translation2d targetField) {
    return targetField.minus(robotPoseField.getTranslation());
  }
}
