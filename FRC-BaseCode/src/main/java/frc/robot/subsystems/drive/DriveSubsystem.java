package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.io.gryo.GryoIO;
import frc.robot.io.gryo.GryoIO.GryoIOValues;
import frc.robot.io.vision.VisionIO.VisionPoseMeasurement;
import frc.robot.subsystems.drive.DriveSubsystemConstants.DriveConstants;
import frc.robot.subsystems.vision.VisionSubsystemConstants;

public class DriveSubsystem extends SubsystemBase {
    private SwerveModule m_frontLeft;
    private SwerveModule m_frontRight;
    private SwerveModule m_rearLeft;
    private SwerveModule m_rearRight;

    // The gyro sensor
    private GryoIO m_gyro;
    private GryoIOValues m_gryoValues;

    // Odometry class for tracking robot pose
    private SwerveDrivePoseEstimator m_poseEstimator;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem(
        SwerveModule frontLeftModule,
        SwerveModule frontRighModule,
        SwerveModule rearLeftModule,
        SwerveModule rearRightModule,
        GryoIO gryo,
        GryoIOValues gryoValues
    ) {
        // Usage reporting for MAXSwerve template
        HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

        this.m_frontLeft = frontLeftModule;
        this.m_frontRight = frontRighModule;
        this.m_rearLeft = rearLeftModule;
        this.m_rearRight = rearRightModule;
        
        this.m_gyro = gryo;
        this.m_gryoValues = gryoValues;

        // get init values
        this.m_gyro.updateInputs(gryoValues);
        this.m_frontLeft.update();
        this.m_frontRight.update();
        this.m_rearLeft.update();
        this.m_rearRight.update();


        this.m_poseEstimator = new SwerveDrivePoseEstimator(
        DriveConstants.kDriveKinematics,
        Rotation2d.fromRadians(m_gryoValues.position.value.getZ()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
            new Pose2d() //init pose
        );

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();

            // Configure AutoBuilder last
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),
                    config, // The robot configuration
                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                    },
                    this // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }
    }

    @Override
    public void periodic() {
        // Update each individual module
        m_frontLeft.update();
        m_frontRight.update();
        m_rearLeft.update();
        m_rearRight.update();

        // Update gryo values
        m_gyro.updateInputs(m_gryoValues);

        // Update the odometry in the periodic block if robot is stable
        if(Math.abs(m_gryoValues.position.value.getX()) < VisionSubsystemConstants.kMaxRollRadForFusion &&
            Math.abs(m_gryoValues.position.value.getY()) < VisionSubsystemConstants.kMaxPitchRadForFusion
        ) {
            m_poseEstimator.updateWithTime(
                Timer.getFPGATimestamp(),
                Rotation2d.fromRadians(m_gryoValues.position.value.getZ()),
                new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_rearLeft.getPosition(),
                    m_rearRight.getPosition()
                }
            );
        }
    }

    public Command pathfindToPose(Pose2d target) {
        PathConstraints constraints = new PathConstraints(
            DriveSubsystemConstants.AutoConstants.kMaxSpeedMetersPerSecond,
            DriveSubsystemConstants.AutoConstants.kMaxAccelerationMetersPerSecondSquared,          // max vel m/s, max accel m/s^2
            DriveSubsystemConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,
            DriveSubsystemConstants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared, // max ang vel rad/s, max ang accel rad/s^2
            12.0,
            false
        );

        return AutoBuilder.pathfindToPose(target, constraints, 0.0);
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kDriveKinematics.toChassisSpeeds(
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_rearLeft.getState(),
            m_rearRight.getState()
        );
    }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        SwerveModuleState[] states = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
        setModuleStates(states);
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_poseEstimator.getEstimatedPosition();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetPose(Pose2d pose) {
        m_poseEstimator.resetPosition(
            Rotation2d.fromRadians(m_gryoValues.position.value.getZ()),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_rearLeft.getPosition(),
                m_rearRight.getPosition()
            },
            pose
        );
    }

    public void addVisionMeasurement(VisionPoseMeasurement m) {
        m_poseEstimator.addVisionMeasurement(m.pose(), m.timestampSeconds(), m.stdDevs());
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the
     *                      field.
     */
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
        // Convert the commanded speeds into the correct units for the drivetrain
        double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
        double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

        var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                    Rotation2d.fromRadians(m_gryoValues.position.value.getZ()))
                : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
        SwerveDriveKinematics.desaturateWheelSpeeds(
            swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    /**
     * Sets the wheels into an X formation to prevent movement.
     */
    public void setX() {
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(
            desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_rearLeft.setDesiredState(desiredStates[2]);
        m_rearRight.setDesiredState(desiredStates[3]);
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        m_frontLeft.resetEncoders();
        m_rearLeft.resetEncoders();
        m_frontRight.resetEncoders();
        m_rearRight.resetEncoders();
    }

    /** Sets the heading of the robot. */
    public void setHeading(double heading) {
        m_gyro.setYawPosition(heading);
    }

    /**
     * Returns the heading of the robot.
     *
     * @return
     */
    public Rotation3d getHeading() {
        return m_gryoValues.position.value;
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, radians per second
     */
    public double getTurnRate() {
        return m_gryoValues.velocityYawRadPerSec.value;
    }
    
}
