package frc.robot.utils.yearSpecific;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.utils.GeometryUtil;

public final class TurretShooter {
    private TurretShooter() {}

    /**
     * "Stationary" shot request: target location in field coords, and
     * the desired ground speed of the ball toward that target if the robot was not moving.
     */
    public record StationaryShot(Translation2d goalFieldPos, double desiredBallGroundSpeedMps) {}

    /**
     * Output of full compensation: where to aim (field angle) and what shot speed is required.
     */
    public record CompensatedShot(Rotation2d aimFieldAngle, double requiredShotSpeedMps) {}

    public record CompensatedShotRobotRelative(Rotation2d aimRobotAngle, double requiredShotSpeedMps) {}

    
    public static double getStationaryShotMPSFromPose(double distanceMeters, double goalHeightMeters, double initHeightMeters, double shotAngleRad) {
        double h = goalHeightMeters - initHeightMeters;
        double d = distanceMeters;
        double g = 9.81;

        double cos = Math.cos(shotAngleRad);
        double tan = Math.tan(shotAngleRad);

        double denom = 2 * cos * cos * (d * tan - h);
        if(denom <= 0) {
            return 0; // Impossible shot
        }

        return Math.sqrt((g * d * d) / denom);
    }

    public static double getOptimalShotAngle(double distanceMeters, double goalHeightMeters, double initHeightMeters) {
        double d = distanceMeters;
        double h = goalHeightMeters - initHeightMeters;
        return Math.atan(d / (h + Math.sqrt(d*d + h*h)));
    }

    /**
     * Full vector compensation.
     *
     * v_ball_field = v_shot_field + v_robot_field
     * so v_shot_field = v_ball_field - v_robot_field
     */
    public static CompensatedShot compensateForRobotMovement(
        Pose2d robotPoseField,
        Translation2d robotVelFieldMps,
        StationaryShot stationaryShot
    ) {
        Translation2d robotPos = robotPoseField.getTranslation();
        Translation2d goalPos = stationaryShot.goalFieldPos();

        // Unit vector from robot -> goal
        Translation2d uToGoal = GeometryUtil.unitTo(robotPos, goalPos);

        // If goal is basically at robot position, just aim forward and keep speed
        if (uToGoal.getNorm() < 1e-9) {
            return new CompensatedShot(robotPoseField.getRotation(), stationaryShot.desiredBallGroundSpeedMps());
        }

        // Desired ball velocity on the field, toward the goal
        Translation2d desiredBallVelField = uToGoal.times(stationaryShot.desiredBallGroundSpeedMps());

        // Required shot velocity relative to robot (still expressed in field frame)
        Translation2d requiredShotVelField = desiredBallVelField.minus(robotVelFieldMps);

        return new CompensatedShot(requiredShotVelField.getAngle(), requiredShotVelField.getNorm());
    }

    /** Convert the field aim angle into a turret-relative-to-robot angle. */
    public static Rotation2d turretAngleRobot(
        Pose2d robotPoseField,
        CompensatedShot shot
    ) {
        return GeometryUtil.fieldAngleToRobot(shot.aimFieldAngle(), robotPoseField.getRotation());
    }

    public static CompensatedShotRobotRelative getFinalShot(
        double distanceMeters,
        double goalHeightMeters,
        double initHeightMeters,
        Pose2d robotPoseField,
        Translation2d robotVelFieldMps,
        Translation2d goalFieldPose
    ) {
        double shotAngleRad = getOptimalShotAngle(distanceMeters, goalHeightMeters, initHeightMeters);
        StationaryShot stationaryShot = new StationaryShot(
            goalFieldPose,
            getStationaryShotMPSFromPose(distanceMeters, goalHeightMeters, initHeightMeters, shotAngleRad)
        );

        CompensatedShot compensatedShot = compensateForRobotMovement(robotPoseField, robotVelFieldMps, stationaryShot);

        return new CompensatedShotRobotRelative(
            turretAngleRobot(robotPoseField, compensatedShot),
            compensatedShot.requiredShotSpeedMps
        );
    }

    public static CompensatedShotRobotRelative getFinalShot(
        double distanceMeters,
        double goalHeightMeters,
        double initHeightMeters,
        Pose2d robotPoseField,
        Translation2d robotVelFieldMps,
        Translation2d goalFieldPose,
        double shotAngleRad
    ) {
        StationaryShot stationaryShot = new StationaryShot(
            goalFieldPose,
            getStationaryShotMPSFromPose(distanceMeters, goalHeightMeters, initHeightMeters, shotAngleRad)
        );

        CompensatedShot compensatedShot = compensateForRobotMovement(robotPoseField, robotVelFieldMps, stationaryShot);

        return new CompensatedShotRobotRelative(
            turretAngleRobot(robotPoseField, compensatedShot),
            compensatedShot.requiredShotSpeedMps
        );
    }
}
