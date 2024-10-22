package org.firstinspires.ftc.teamcode.util;

public class Constants {
    // ----ARM VARIABLES----
    public static final int ARM_UP_POSITION = -1000;
    public static final int ARM_DOWN_POSITION = 0;
    public static final double ARM_MOTOR_STRENGTH = 0.9;
    public static final double WRIST_ON_WALL =  0.6;
    public static final double CLAW_OPEN_POS = 0.5;
    public static final double CLAW_CLOSED_POS = 0.9;
    public static final double WRIST_INC = 0.01;
    public static final double WRIST_MAX = 0.8;
    public static final double WRIST_ON_GROUND = 0.36;
    public static final double WRIST_MIN = 0.0;
    public static final double ROLL_MAX = 0.72;
    public static final double ROLL_MIN = 0;
    public static final double ROLL_INC = 0.03;
    public static final double ARM_MOTOR_kP = 0.05;
    public static final double ARM_MOTOR_TOLERANCE = 13.6;

    // -------DRIVE--------
    // -------DRIVE--------
    public static final double MOTOR_MAX_SPEED = 1;
    public static final double SWERVE_ENCODER_COUNTS_PER_REV = 2047.136; // Single revolution encoder ticks
    public static final double SWERVE_ENCODER_COUNTS_PER_INCH =  260.649; // Encoder Ticks per Inch
    public static final double SWERVE_WHEEL_ROT_MULTIPLIER = 3;
    public static final double APRIL_TAG_DISTANCE_TARGET = 5;
    public static final double APRIL_TAG_PRECISION = 10;
    public static final double APRIL_TAG_MAX_SPEED = 0.3;

    // Generalized minimum input for the robot to respond to controller input
    public static final double INPUT_THRESHOLD = 0.1;
    // Generalized minimum angle difference for the robot to respond to an autonomous movement command
    public static final double ANGLE_THRESHOLD = 3.0;
    // Generalized minimum position difference for the robot to respond to an autonomous movement command
    public static final double DISTANCE_THRESHOLD = 1.0;
    public static final double DEFAULT_TIMEOUT = 10.0;

    public static final double KP = 0.2;

    // ELEVATOR
    public static final double LOCK_OFF_POSITION = 0.0;
    public static final double LOCK_ON_POSITION = 1.0;
    public static final double ELEVATOR_MAX_SPEED = 0.9;

    // LIFT
    public static final int LOW_HEIGHT = 0;
    public static final int MID_HEIGHT = 250;
    public static final int HIGH_HEIGHT = 500;
    public static final double LIFT_MAX_SPEED = 0.9;



    // ----- HARDWARE MAP NAMES ------
    // -------- SENSOR NAMES ---------
    public static final String IMU_NAME = "imu";
    public static final String REAR_DIST_NAME = "rear_distance";
    public static final String RIGHT_DIST_NAME = "right_distance";
    public static final String LEFT_DIST_NAME = "left_distance";
    public static final String WEBCAM_NAME = "Webcam 1";

    // --------- MOTOR NAMES ---------
    public static final String LEFT_FRONT_NAME = "leftFront";
    public static final String RIGHT_FRONT_NAME = "rightFront";
    public static final String LEFT_BACK_NAME = "leftBack";
    public static final String RIGHT_BACK_NAME = "rightBack";
    public static final String ARM_MOTOR_NAME = "arm";
    public static final String LIFT_MOTOR_NAME = "lift";
    public static final String SHOULDER_MOTOR_NAME = "Shoulder";
    // encoder aliases
    public static final String LEFT_ODOMETRY_NAME = LEFT_FRONT_NAME;
    // TODO: Right odom should be right front
    // center should be right back
    public static final String RIGHT_ODOMETRY_NAME = LEFT_BACK_NAME;
    public static final String CENTER_ODOMETRY_NAME = RIGHT_FRONT_NAME;

    // --------- SERVO NAMES ---------
    public static final String WRIST_SERVO_NAME = "Wrist";
    public static final String OPEN_SERVO_NAME = "open";
    public static final String ROLL_SERVO_NAME = "roll";
    public static final String LAUNCHER_SERVO_NAME = "launcher";
    public static final String ELEVATOR_LEFT_SERVO_NAME = "elevLeft";
    public static final String ELEVATOR_RIGHT_SERVO_NAME = "elevRight";
    public static final String LIFT_BASKET_SERVO_NAME = "basket";

    // -----Other------

    public static final String LED_CONTROLLER = "ledLight";
    public static final String CLAW_SERVO_NAME = "Claw";
}
