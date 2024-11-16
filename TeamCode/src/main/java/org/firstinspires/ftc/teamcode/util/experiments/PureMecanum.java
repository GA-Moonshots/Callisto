package org.firstinspires.ftc.teamcode.util.experiments;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;
import org.firstinspires.ftc.teamcode.util.Constants;

public class PureMecanum extends SubsystemBase {
    private MecanumDrive mecanumDrive;

    // SUBSYSTEM ASSETS
    private final Callisto robot;
    private Telemetry telemetry;
    public final IMU imu;

    // GYRO VARIABLES
    private boolean isTargetSet = false;
    private double fieldCentricTarget = 0.0d;
    private boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private double gyroTarget = 0.0d;

    // MOTORS
    private final MotorEx leftFront, leftBack, rightFront, rightBack;

    // CAMERA
    Camera camera;

    public PureMecanum(Callisto robot){
        // convenient references
        this.robot = robot;

        // init our camera
        camera = new Camera(this.robot, this.robot.opMode.telemetry);

        // init our motors
        leftFront = new MotorEx(this.robot.opMode.hardwareMap, Constants.LEFT_FRONT_NAME);
        leftBack = new MotorEx(this.robot.opMode.hardwareMap, Constants.LEFT_BACK_NAME);
        rightFront = new MotorEx(this.robot.opMode.hardwareMap, Constants.RIGHT_FRONT_NAME);
        rightBack = new MotorEx(this.robot.opMode.hardwareMap, Constants.RIGHT_BACK_NAME);

        mecanumDrive = new MecanumDrive(leftFront, rightFront, leftBack, rightBack);

        // TODO: instantiate distance sensors

        imu = this.robot.opMode.hardwareMap.get(IMU.class, Constants.IMU_NAME);
        this.resetFieldCentricTarget();
    }

    /**
     * Translates desired motion into mecanum commands
     * @param forward negative is forward
     * @param strafe lateral movement
     * @param turn positive is clockwise
     */
    public void drive(double forward, double strafe, double turn) {
        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double diff = fieldCentricTarget - getZAngle();
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(diff)) + strafe * Math.sin(Math.toRadians(diff));
            strafe = (-temp * Math.sin(Math.toRadians(diff)) + strafe * Math.cos(Math.toRadians(diff)));
            if(telemetry != null) {
                telemetry.addData("Mode", "Field Centric");
                telemetry.update();
            }
        } else if(telemetry != null)
            telemetry.addData("Mode", "Robot Centric");

        isGyroLocked = turn <= Constants.INPUT_THRESHOLD;
        if(isGyroLocked && !isTargetSet) {
            gyroTarget = getYAngle();
            isTargetSet = true;
        } else if(!isGyroLocked) {
            isTargetSet = false;
        }

        // I'm tired of figuring out the input problems so the inputs are still in flight stick mode
        // Meaning forward is reversed
        // The boost values should match the turn
        // Since the drive is a diamond wheel pattern instead of an X, it reverses the strafe.
        double leftFrontPower =  -forward - strafe - turn;
        double rightFrontPower = -forward + strafe + turn;
        double leftBackPower = forward - strafe + turn;
        double rightBackPower = -forward - strafe + turn;

        if(telemetry != null)
            telemetry.addData("Motors", "(%.2f, %.2f, %.2f, %.2f)",
                    leftFrontPower, leftBackPower, rightBackPower, rightFrontPower);

        drive(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );
        //mecanumDrive.driveRobotCentric(strafe, forward, turn);
    }


    /**
     * Clips and executes given motor speeds
     */
    protected void drive(double m1, double m2, double m3, double m4) {
        leftFront.set(Range.clip(m1, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightFront.set(Range.clip(m2, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        leftBack.set(Range.clip(m3, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightBack.set(Range.clip(m4, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
    }


    // --- FIELD CENTRIC HELPERS ---
    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }
    public void resetFieldCentricTarget() {
        fieldCentricTarget = getZAngle();
    }

    /**
     * @return the difference between the robot's current angle and the Zero Angle
     */
    public double getAngleDifferenceFromZero() {
        double currentAngle = robot.mecanum.getZAngle(); // Get current angle
        double angleDifference = currentAngle - fieldCentricTarget; // Calculate difference

        // if the difference is greater than or equal to 360, subtract 360 to keep it within -180 to 180
        if (angleDifference >= 360) {
            angleDifference -= 360;
        }

        // if the difference is less than -360, add 360 to keep it within -180 to 180
        if (angleDifference < -360) {
            angleDifference += 360;
        }

        return angleDifference; // Return the normalized angle difference
    }

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }
    /**
     *
     * @return the X angle of the internal IMU in the control panel.
     */
    public double getXAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).firstAngle;
    }

    /**
     *
     * @return the Y angle of the internal IMU in the control panel.
     */
    public double getYAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).secondAngle;
    }



}
