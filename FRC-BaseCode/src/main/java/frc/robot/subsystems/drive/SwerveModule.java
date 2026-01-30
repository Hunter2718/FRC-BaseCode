package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.io.encoder.EncoderIO;
import frc.robot.io.encoder.EncoderIO.EncoderIOValues;
import frc.robot.io.motor.MotorGroup;
import frc.robot.io.motor.MotorIO.MotorIOValues;

public class SwerveModule {
    private MotorGroup m_drivingMotors;
    private MotorGroup m_turningMotors;
    private List<MotorIOValues> drivingMotorValues;
    private List<MotorIOValues> turningMotorValues;

    private EncoderIO m_drivingEncoder;
    private EncoderIO m_turningEncoder;
    private EncoderIOValues drivingEncoderValues;
    private EncoderIOValues turningEncoderValues;

    private double m_chassisAngularOffset = 0;
    private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

    public SwerveModule(
        MotorGroup drivingMotors,
        MotorGroup turningMotors,
        EncoderIO drivingEncoder,
        EncoderIO turningEncoder,
        double chassisAngularOffset
    ) {
        this.m_drivingMotors = drivingMotors;
        this.m_turningMotors = turningMotors;

        this.drivingMotorValues = new ArrayList<>(drivingMotors.getSize());
        this.turningMotorValues = new ArrayList<>(turningMotors.getSize());
        this.drivingEncoderValues = new EncoderIOValues();
        this.turningEncoderValues = new EncoderIOValues();

        this.m_drivingEncoder = drivingEncoder;
        this.m_turningEncoder = turningEncoder;


        m_chassisAngularOffset = chassisAngularOffset;
        m_desiredState.angle = new Rotation2d(turningEncoderValues.positionRad.value);
        m_drivingEncoder.setPosition(0);
    }

    public void update() {
        m_drivingMotors.updateInputs(drivingMotorValues);
        m_turningMotors.updateInputs(turningMotorValues);
        m_drivingEncoder.updateInputs(drivingEncoderValues);
        m_turningEncoder.updateInputs(turningEncoderValues);
    }

    /**
     * Returns the current state of the module.
     *
     * @return The current state of the module.
     */
    public SwerveModuleState getState() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        double motorRadPerSec = drivingEncoderValues.velocityRadPerSec.value;
        double wheelRadPerSec = motorRadPerSec / DriveSubsystemConstants.ModuleConstants.kDriveGearRatio;
        double speedMps = wheelRadPerSec * DriveSubsystemConstants.ModuleConstants.kWheelRadiusM;

        // chassis-relative module angle = raw - offset
        Rotation2d angle = Rotation2d.fromRadians(
            turningEncoderValues.positionRad.value - m_chassisAngularOffset
        );

        return new SwerveModuleState(speedMps, angle);
    }

    /**
     * Returns the current position of the module.
     *
     * @return The current position of the module.
     */
    public SwerveModulePosition getPosition() {
        // Apply chassis angular offset to the encoder position to get the position
        // relative to the chassis.
        double motorRad = drivingEncoderValues.positionRad.value;
        double wheelRad = motorRad / DriveSubsystemConstants.ModuleConstants.kDriveGearRatio;
        double distanceM = wheelRad * DriveSubsystemConstants.ModuleConstants.kWheelRadiusM;

        Rotation2d angle = Rotation2d.fromRadians(
            turningEncoderValues.positionRad.value - m_chassisAngularOffset
        );

        return new SwerveModulePosition(distanceM, angle);
    }

    /**
     * Sets the desired state for the module.
     *
     * @param desiredState Desired state with speed and angle.
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // 1) Apply chassis angular offset (REV template behavior)
        SwerveModuleState corrected = new SwerveModuleState(
            desiredState.speedMetersPerSecond,
            desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset))
        );

        // 2) Optimize (instance method; mutates 'corrected')
        Rotation2d currentAngleEncoderFrame =
            Rotation2d.fromRadians(turningEncoderValues.positionRad.value);

        corrected.optimize(currentAngleEncoderFrame);

        // 3) Drive velocity (closed-loop): m/s -> motor rad/s
        double wheelRadPerSec = corrected.speedMetersPerSecond / DriveSubsystemConstants.ModuleConstants.kWheelRadiusM;
        double motorRadPerSec = wheelRadPerSec * DriveSubsystemConstants.ModuleConstants.kDriveGearRatio;

        m_drivingMotors.setVelocityRadPerSec(motorRadPerSec, 0.0);

        // 4) Turn position (closed-loop): module rad -> motor rad
        double moduleAngleRad = corrected.angle.getRadians();
        double motorAngleRad = moduleAngleRad * DriveSubsystemConstants.ModuleConstants.kTurnGearRatio;

        m_turningMotors.setPositionRad(motorAngleRad, 0.0);

        m_desiredState = desiredState;
    }



    /** Zeroes all the SwerveModule encoders. */
    public void resetEncoders() {
        m_drivingEncoder.setPosition(0);
    }
}
