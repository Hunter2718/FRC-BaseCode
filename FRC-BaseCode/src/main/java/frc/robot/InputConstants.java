package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class InputConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriverControllerStickDeadband = 0.08;
    public static final double kDriverControllerTriggerDeadband = 0.25;

    // Specific Button Values
    public static final int kDriverControllerButtonXWheels = XboxController.Button.kX.value;
    public static final int kDriverControllerButtonResetOdometry = XboxController.Button.kY.value;



    public static final int kOpperatorJoystickPort = 1;


    // Specific Button Values
    public static final int kOpperatorJoystickButtonIntakeSpin = 1;
    public static final int kOpperatorJoystickButtonIntakeLow = 2;
    public static final int kOpperatorJoystickButtonIntakeHigh = 3;
    public static final int kOpperatorJoystickButtonShooterSpin = 0;



    public static final int kButtonBoardPort = 2;


    // Spicific Button Values
    public static final int kButtonBoardShooterLowButton = 0;
    public static final int kButtonBoardShooterHighButton = 0;
}
