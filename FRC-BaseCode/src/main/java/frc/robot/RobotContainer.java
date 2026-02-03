// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.StopAll;
import frc.robot.commands.PositionPiece.PositionPieceTeleop;
import frc.robot.commands.drive.TeleopDrive;
import frc.robot.commands.drive.XWheels;
import frc.robot.commands.flyWheel.FlyWheelTeleop;
import frc.robot.commands.vision.AlignQuest;
import frc.robot.commands.vision.ResetPoseAndAlignQuest;
import frc.robot.io.encoder.AbsoluteEncoderIO;
import frc.robot.io.encoder.RelativeEncoderIO;
import frc.robot.io.gryo.Pideon2IO;
import frc.robot.io.motor.MotorGroup;
import frc.robot.io.motor.SparkFlexIO;
import frc.robot.io.motor.SparkMaxIO;
import frc.robot.io.vision.VisionIO.VisionIOValues;
import frc.robot.io.vision.VisionIOLimelight;
import frc.robot.io.vision.VisionIOLimelight.Mode;
import frc.robot.io.vision.VisionIOQuestNav;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystemConstants;
import frc.robot.subsystems.drive.SwerveModule;
import frc.robot.subsystems.flyWheel.FlyWheelSubsystem;
import frc.robot.subsystems.flyWheel.FlyWheelSubsystemConstants;
import frc.robot.subsystems.positionPiece.PositionPieceSubsystem;
import frc.robot.subsystems.positionPiece.PositionPieceSubsystemConstants;
import frc.robot.subsystems.vision.VisionSubsystem;
import frc.robot.subsystems.vision.VisionSubsystemConstants;
import gg.questnav.questnav.QuestNav;

public class RobotContainer {
  // Auto Chooser
  private final SendableChooser<Command> autoChooser;

  // Controllers
  private XboxController m_driverController;
  private Joystick m_opperatorJoystick;
  private GenericHID m_buttonBoard;

  // Subsystems
  private DriveSubsystem m_drive;
  @SuppressWarnings("Unused")
  private VisionSubsystem m_vision;
  private FlyWheelSubsystem m_intake;
  private PositionPieceSubsystem m_intakePivotJoint;
  private FlyWheelSubsystem m_shooter;
  private PositionPieceSubsystem m_turret;

  // Intake
  private SparkMax intakeSparkMax;

  // Intake Pivot Joint
  private SparkMax intakePivotJointSparkMax;

  // Shooter
  private SparkFlex shooterFlex;

  // Turret
  private SparkMax turretSparkMax;

  // Swerve Modules
  private SwerveModule flSwerveModule;
  private SwerveModule frSwerveModule;
  private SwerveModule rlSwerveModule;
  private SwerveModule rrSwerveModule;

  
  // Drive
  private SparkMax driveFLTurnSparkMax;
  private SparkMax driveFRTurnSparkMax;
  private SparkMax driveRLTurnSparkMax;
  private SparkMax driveRRTurnSparkMax;
  private SparkFlex driveFLDriveFlex;
  private SparkFlex driveFRDriveFlex;
  private SparkFlex driveRLDriveFlex;
  private SparkFlex driveRRDriveFlex;

  // Gryo
  private Pigeon2 pigeon2;

  // Vision
  private VisionIOLimelight leftLL;
  private VisionIOLimelight rightLL;
  private VisionIOQuestNav quest;
  private QuestNav questNav;


  public RobotContainer() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Build Gryo
    pigeon2 = new Pigeon2(DriveSubsystemConstants.DriveConstants.kGyroPort);


    // Build Drive
    // Turn Motors
    // Front Left
    driveFLTurnSparkMax = new SparkMax(DriveSubsystemConstants.DriveConstants.kFrontLeftTurningCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveFLTurnSparkMax.configure(RevConfigs.MAXSwerveModule.turningConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Front Right
    driveFRTurnSparkMax = new SparkMax(DriveSubsystemConstants.DriveConstants.kFrontRightTurningCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveFRTurnSparkMax.configure(RevConfigs.MAXSwerveModule.turningConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Rear Left
    driveRLTurnSparkMax = new SparkMax(DriveSubsystemConstants.DriveConstants.kRearLeftTurningCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveRLTurnSparkMax.configure(RevConfigs.MAXSwerveModule.turningConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Rear Right
    driveRRTurnSparkMax = new SparkMax(DriveSubsystemConstants.DriveConstants.kRearRightTurningCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveRRTurnSparkMax.configure(RevConfigs.MAXSwerveModule.turningConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Drive Motors
    // Front Left
    driveFLDriveFlex = new SparkFlex(DriveSubsystemConstants.DriveConstants.kFrontLeftDrivingCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveFLDriveFlex.configure(RevConfigs.MAXSwerveModule.drivingConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Front Right
    driveFRDriveFlex = new SparkFlex(DriveSubsystemConstants.DriveConstants.kFrontRightDrivingCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveFRDriveFlex.configure(RevConfigs.MAXSwerveModule.drivingConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Rear Left
    driveRLDriveFlex = new SparkFlex(DriveSubsystemConstants.DriveConstants.kRearLeftDrivingCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveRLDriveFlex.configure(RevConfigs.MAXSwerveModule.drivingConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Rear Right
    driveRRDriveFlex = new SparkFlex(DriveSubsystemConstants.DriveConstants.kRearRightDrivingCanId, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    driveRRDriveFlex.configure(RevConfigs.MAXSwerveModule.drivingConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    // Modules
    flSwerveModule = new SwerveModule(

      new MotorGroup(
        List.of(
          new SparkFlexIO(
            driveFLDriveFlex
          )
        ),
        0,
        true
      ),

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            driveFLTurnSparkMax
          )
        ),
        0,
        true
      ),

      new RelativeEncoderIO(
        driveFLDriveFlex.getEncoder()
      ),

      new AbsoluteEncoderIO(
        driveFLTurnSparkMax.getAbsoluteEncoder()
      ),

      DriveSubsystemConstants.DriveConstants.kFrontLeftChassisAngularOffset
    );


    frSwerveModule = new SwerveModule(

      new MotorGroup(
        List.of(
          new SparkFlexIO(
            driveFRDriveFlex
          )
        ),
        0,
        true
      ),

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            driveFRTurnSparkMax
          )
        ),
        0,
        true
      ),

      new RelativeEncoderIO(
        driveFRDriveFlex.getEncoder()
      ),

      new AbsoluteEncoderIO(
        driveFRTurnSparkMax.getAbsoluteEncoder()
      ),

      DriveSubsystemConstants.DriveConstants.kFrontRightChassisAngularOffset
    );


    rlSwerveModule = new SwerveModule(

      new MotorGroup(
        List.of(
          new SparkFlexIO(
            driveRLDriveFlex
          )
        ),
        0,
        true
      ),

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            driveRLTurnSparkMax
          )
        ),
        0,
        true
      ),

      new RelativeEncoderIO(
        driveRLDriveFlex.getEncoder()
      ),

      new AbsoluteEncoderIO(
        driveRLTurnSparkMax.getAbsoluteEncoder()
      ),

      DriveSubsystemConstants.DriveConstants.kBackLeftChassisAngularOffset
    );


    rrSwerveModule = new SwerveModule(

      new MotorGroup(
        List.of(
          new SparkFlexIO(
            driveRRDriveFlex
          )
        ),
        0,
        true
      ),

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            driveRRTurnSparkMax
          )
        ),
        0,
        true
      ),

      new RelativeEncoderIO(
        driveRRDriveFlex.getEncoder()
      ),

      new AbsoluteEncoderIO(
        driveRRTurnSparkMax.getAbsoluteEncoder()
      ),

      DriveSubsystemConstants.DriveConstants.kBackRightChassisAngularOffset
    );


    m_drive = new DriveSubsystem(

      flSwerveModule,

      frSwerveModule,

      rlSwerveModule,

      rrSwerveModule,

      new Pideon2IO(
        pigeon2
      )
    );

  
    
    // Build Vision
    questNav = new QuestNav();
    leftLL = new VisionIOLimelight("limelight-left", Mode.MEGATAG2_WPIBLUE, VisionSubsystemConstants.llStd);
    rightLL = new VisionIOLimelight("limelight-right", Mode.MEGATAG2_WPIBLUE, VisionSubsystemConstants.llStd);
    quest = new VisionIOQuestNav("questnav", questNav, VisionSubsystemConstants.robotToQuest, VisionSubsystemConstants.questStd);

    m_vision = new VisionSubsystem(

      List.of(
        leftLL,
        rightLL,
        quest
      ),

      List.of(
        new VisionIOValues(),
        new VisionIOValues(),
        new VisionIOValues()
      ),

      () -> m_drive.getHeading(),

      (m) -> m_drive.addVisionMeasurement(m),

      () -> m_drive.getPose(),

      List.of(
        (pose) -> quest.resetFromRobotPose(pose)
      )

    );



    // Intake
    intakeSparkMax = new SparkMax(FlyWheelSubsystemConstants.kIntakeMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    intakeSparkMax.configure(RevConfigs.Intake.intakeConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    m_intake = new FlyWheelSubsystem(

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            intakeSparkMax
          )
        ),
        0,
        true
      ),

      FlyWheelSubsystemConstants.kIntakeGearRatio,

      FlyWheelSubsystemConstants.kIntakeFFVolts,

      FlyWheelSubsystemConstants.kIntakeAtSpeedToleranceRadPerSec
    );

    

    // Intake Pivot Joint
    intakePivotJointSparkMax = new SparkMax(PositionPieceSubsystemConstants.kIntakePivotJointMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    intakePivotJointSparkMax.configure(RevConfigs.Intake.intakePivotJointConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    m_intakePivotJoint = new PositionPieceSubsystem(

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            intakePivotJointSparkMax
          )
        ),
        0,
        true
      ),

      List.of(
        new AbsoluteEncoderIO(
          intakePivotJointSparkMax.getAbsoluteEncoder()
        )
      ),

      PositionPieceSubsystemConstants.kIntakePivotJointGearRatio,

      PositionPieceSubsystemConstants.kIntakePivotJointManualMaxVolts,

      PositionPieceSubsystemConstants.kMinIntakePivotJointRad,

      PositionPieceSubsystemConstants.kMaxIntakePivotJointRad,

      PositionPieceSubsystemConstants.kIntakePivotJointFFVolts,

      PositionPieceSubsystemConstants.kIntakePivotJointAtGoalToleranceRad
    );



    // Shooter
    shooterFlex = new SparkFlex(FlyWheelSubsystemConstants.kShooterMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    shooterFlex.configure(RevConfigs.TurretShooter.shooterConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    m_shooter = new FlyWheelSubsystem(

      new MotorGroup(
        List.of(
          new SparkFlexIO(
            shooterFlex
          )
        ),
        0,
        true
      ),

      FlyWheelSubsystemConstants.kShooterGearRatio,

      FlyWheelSubsystemConstants.kShooterFFVolts,

      FlyWheelSubsystemConstants.kShooterAtSpeedToleranceRadPerSec
    );



    // Turret
    turretSparkMax = new SparkMax(PositionPieceSubsystemConstants.kTurretMotorPort, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);
    turretSparkMax.configure(RevConfigs.TurretShooter.turretConfig, com.revrobotics.ResetMode.kResetSafeParameters, com.revrobotics.PersistMode.kPersistParameters);

    m_turret = new PositionPieceSubsystem(

      new MotorGroup(
        List.of(
          new SparkMaxIO(
            turretSparkMax
          )
        ),
        0,
        true
      ),

      List.of(
        new AbsoluteEncoderIO(
          turretSparkMax.getAbsoluteEncoder()
        )
      ),

      PositionPieceSubsystemConstants.kTurretGearRatio,

      PositionPieceSubsystemConstants.kTurretManualMaxVolts,

      PositionPieceSubsystemConstants.kMinTurretRad,

      PositionPieceSubsystemConstants.kMaxTurretRad,

      PositionPieceSubsystemConstants.kTurretFFVolts,

      PositionPieceSubsystemConstants.kTurretAtGoalToleranceRad
    );

    
    
    // Controller
    m_driverController = new XboxController(InputConstants.kDriverControllerPort);
    m_opperatorJoystick = new Joystick(InputConstants.kOpperatorJoystickPort);
    m_buttonBoard = new GenericHID(InputConstants.kButtonBoardPort);

    configureBindings();
  }


  private void configureBindings() {
    // Auto Bindings
    configureAutoCommands();

    // Default Bindings
    m_drive.setDefaultCommand(
      new TeleopDrive(
        m_drive,
        () -> -MathUtil.applyDeadband(m_driverController.getLeftY(), InputConstants.kDriverControllerStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getLeftX(), InputConstants.kDriverControllerStickDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRightX(), InputConstants.kDriverControllerStickDeadband),
        () -> !m_driverController.getRightBumperButton(),
        () -> m_driverController.getLeftTriggerAxis() > InputConstants.kDriverControllerTriggerDeadband,
        () -> m_driverController.getRightTriggerAxis() > InputConstants.kDriverControllerTriggerDeadband
      )
    );

    m_intake.setDefaultCommand(
      new FlyWheelTeleop(
        m_intake,
        () -> m_opperatorJoystick.getRawButton(InputConstants.kOpperatorJoystickButtonIntakeSpin),
        () -> m_opperatorJoystick.getRawButton(InputConstants.kOpperatorJoystickButtonIntakeLow),
        () -> m_opperatorJoystick.getRawButton(InputConstants.kOpperatorJoystickButtonIntakeHigh),
        FlyWheelSubsystemConstants.kIntakeWheelLowRadPerSec,
        FlyWheelSubsystemConstants.kIntakeWheelHighRadPerSec,
        FlyWheelSubsystemConstants.kIntakeWheelNormalRadPerSec
      )
    );

    m_intakePivotJoint.setDefaultCommand(
      new PositionPieceTeleop(
        m_intakePivotJoint,
        () -> m_opperatorJoystick.getY(),
        PositionPieceSubsystemConstants.kIntakePivotJointManualMaxVolts
      )
    );

    m_shooter.setDefaultCommand(
      new FlyWheelTeleop(
        m_shooter,
        () -> m_opperatorJoystick.getRawButton(InputConstants.kOpperatorJoystickButtonShooterSpin),
        () -> m_buttonBoard.getRawButton(InputConstants.kButtonBoardShooterLowButton),
        () -> m_buttonBoard.getRawButton(InputConstants.kButtonBoardShooterHighButton),
        FlyWheelSubsystemConstants.kShooterWheelLowRadPerSec,
        FlyWheelSubsystemConstants.kShooterWheelHighRadPerSec,
        FlyWheelSubsystemConstants.kShooterWheelNormalRadPerSec
      )
    );

    m_turret.setDefaultCommand(
      new PositionPieceTeleop(
        m_turret,
        () -> m_opperatorJoystick.getZ(),
        PositionPieceSubsystemConstants.kTurretManualMaxVolts
      )
    );

    configureTeleAutoCommands();
  }


  private void configureTeleAutoCommands() {
    // Tle-Auto (button) Bindings
    new JoystickButton(m_driverController, InputConstants.kDriverControllerButtonXWheelsButton)
      .whileTrue(
        new XWheels(m_drive)
      );

    new JoystickButton(m_buttonBoard, InputConstants.kButtonBoardResetPoseAndAlignQuestButton)
      .onTrue(
        new ResetPoseAndAlignQuest(
          m_drive,
          m_vision,
          m_drive.getPose() // Change to know pose (Ex. start)
        )
      );

    new JoystickButton(m_driverController, InputConstants.kDriverControllerButtonForceAlignQuestButton)
      .onTrue(
        new AlignQuest(
          m_vision,
          m_drive.getPose()
        )
      );

    // All the subsystems respective stop functions
    new JoystickButton(m_buttonBoard, InputConstants.kButtonBoardStopAllButton)
      .whileTrue(
        new StopAll(
          List.of(
            () -> m_drive.setX(),
            () -> m_turret.stop(),
            () -> m_shooter.stop(),
            () -> m_intake.stop(),
            () -> m_intakePivotJoint.stop()
          )
        )
      );
  }


  private void configureAutoCommands() {
    // Auto Bindings
  }


  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}