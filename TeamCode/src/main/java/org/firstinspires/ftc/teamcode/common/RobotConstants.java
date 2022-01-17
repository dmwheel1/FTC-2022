package org.firstinspires.ftc.teamcode.common;

/**
 * This in Not and opmode
 * List of constants for the robot.
 * Contains Arm, Servo, and Idol Hand Constants.
 *
 * Pull by (RobotConstants.NAME)
 *
 * @author Luke Frazer
 */
public class RobotConstants {

    /* Servo Constants */
    public static final double CLAW_MIN_RANGE       =   0.05;
    public static final double CLAW_MAX_RANGE       =   1.3;
    public static final double CLAW_HOME            =   0.08;
    public static final double CLAW_SPEED           =   0.05;

    /* Idol Hand Constants */
    public static final double IDOLHAND_MIN_RANGE   =   0.05;
    public static final double IDOLHAND_MAX_RANGE   =   1.3;

    /* Autonomous Constants */
    public static final double DRIVE_GEAR_REDUCTION    = .45;     // This is < 1.0 if geared UP (was .667) after it was .625, after that it was .475
                                                                    //  but it changed again because it was going too far
    public static final double WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    public static final double COUNTS_PER_MOTOR_REV    = 1120 ;
    public static final double COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    public static final double FAST_DRIVE              = 0.95;
    public static final double FAST_TURN               = 0.5;
    public static final double STRAFE_SPEED            = 0.5;
    public static final double STRAFE_CONSTANT         = .85;

    public static final double BOOK_IT                 = 0.85;
    public static final double ABOVE_SPEED             = 0.85;
    public static final double DRIVE_SPEED             = 0.65;
    public static final double TURN_SPEED              = 0.55;
    public static final double MEDIUM_SPEED            = 0.45;
    public static final double SLOW_SPEED              = 0.3;
    public static final double CREEP_SPEED             = 0.2;









}
