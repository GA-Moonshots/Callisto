package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.roadrunnerfiles.ThreeDeadWheelLocalizer;
//import org.firstinspires.ftc.teamcode.util.sensors.DistanceSensor;

/**
 * We extend RoadRunner's mecanum drive. That file needs our motor instantiations
 */
public class Mecanum extends RoadRunner {

    // INSTANCE VARIABLES
    private boolean isTargetSet = false;
    private double fieldCentricTarget = 0.0d;
    private boolean isFieldCentric = true;
    private boolean isGyroLocked = false;
    private double gyroTarget = 0.0d;
    private IMU imu;

    // USEFUL REFERENCES
    private final Callisto robot;
    public Telemetry telemetry;

    // CONSTRUCTOR
    public Mecanum(Callisto robot, Pose2d pose) {
        // setup the RoadRunner parent class
        super(robot.opMode.hardwareMap, pose);

        // convenience references
        this.robot = robot;
        this.telemetry = robot.opMode.telemetry;

        // sensors
        this.imu = lazyImu.get();

        this.resetFieldCentricTarget();
    }

    // --- FIELD CENTRIC HELPERS ---
    public void toggleFieldCentric() {
        isFieldCentric = !isFieldCentric;
    }
    public void makeRobotCentric() {
        isFieldCentric = false;
    }
    public void makeFieldCentric() {
        isFieldCentric = true;
    }
    public void resetFieldCentricTarget() {
        fieldCentricTarget = getZAngle();
    }

    public void setGyroLocked(){
        isGyroLocked = true;
    }

    public void setGyroUnlocked(){
        isGyroLocked = true;
    }
    /**
     * Translates desired motion into mecanum commands
     * @param forward negative is forward
     * @param strafe lateral movement
     * @param turn positive is clockwise
     */
    public void drive(double forward, double strafe, double turn) {
        localizer.update();
        // Field Centric adjustment
        if (isFieldCentric) {
            // Learn more:
            // https://www.geogebra.org/m/fmegkksm
            double diff = fieldCentricTarget - getZAngle();
            double temp = forward;
            forward = forward * Math.cos(Math.toRadians(diff)) + strafe * Math.sin(Math.toRadians(diff));
            strafe = (-temp * Math.sin(Math.toRadians(diff)) + strafe * Math.cos(Math.toRadians(diff)));
            if(telemetry != null)
                telemetry.addData("Mode", "Field Centric");
        } else if(telemetry != null)
            telemetry.addData("Mode", "Robot Centric");

        isGyroLocked = turn <= Constants.INPUT_THRESHOLD;
        if(isGyroLocked && !isTargetSet) {
            gyroTarget = getZAngle();
            isTargetSet = true;
        } else if(!isGyroLocked) {
            isTargetSet = false;
        }

        // I'm tired of figuring out the input problems so the inputs are still in flight stick mode
        // Meaning forward is reversed
        // The boost values should match the turn
        // Since the drive is a diamond wheel pattern instead of an X, it reverses the strafe.
//        double leftFrontPower = -forward +strafe + turn;
//        double rightFrontPower = forward + strafe + turn;
//        double leftBackPower = -forward - strafe + turn;
//        double rightBackPower = forward - strafe + turn;

        double leftFrontPower, rightFrontPower;
        double leftBackPower, rightBackPower;

        leftFrontPower =  forward + strafe - turn;
        rightFrontPower = forward - strafe + turn; // this corresponds to the back right tire
        leftBackPower = forward - strafe - turn;
        rightBackPower = forward + strafe + turn; // rn this correspond to the front right tire??

        double powerScale = Constants.MOTOR_MAX_SPEED * Math.max(1,
                Math.max(
                        Math.max(
                                Math.abs(leftFrontPower),
                                Math.abs(leftBackPower)
                        ),
                        Math.max(
                                Math.abs(rightFrontPower),
                                Math.abs(rightBackPower)
                        )
                )
        );

        leftFrontPower /= powerScale;
        leftBackPower /= powerScale;
        rightBackPower /= powerScale;
        rightFrontPower /= powerScale;

        setMotorPowers(
                leftFrontPower,
                rightFrontPower,
                leftBackPower,
                rightBackPower
        );

        telemetry.addData("Angle:", robot.hardwareMap.get(IMU.class,"imu").getRobotYawPitchRollAngles());
    }

    /**
     * Clips and executes given motor speeds
     */
    protected void setMotorPowers(double m1, double m2, double m3, double m4) {
        leftFront.setPower(Range.clip(m1, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightFront.setPower(Range.clip(m2, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        leftBack.setPower(Range.clip(m3, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
        rightBack.setPower(Range.clip(m4, -Constants.MOTOR_MAX_SPEED, Constants.MOTOR_MAX_SPEED));
    }

    public void stop() {
        // Create a PoseVelocity2d object representing zero motion:
        PoseVelocity2d stopMotion = new PoseVelocity2d(new Vector2d(0, 0), 0);

        // Call the setDrivePowers function to apply the zero motion:
        setDrivePowers(stopMotion);
    }

    /**
     * Rotates robot to its imu's 0 degree heading adjusted by fieldCentricTarget. This is not
     * intended to be used with RoadRunner as it doesn't explicitly communicate its intended pose
     */
    public void goToZeroAngle() {
        while (Math.abs(getZAngle() - fieldCentricTarget) >= 2) {
            drive(0.0, 0.0, 0.7 * Math.toRadians(getZAngle() - fieldCentricTarget));
        }
        stop();
    }

    public void postEncoderData() {
        ThreeDeadWheelLocalizer localizer = (ThreeDeadWheelLocalizer)this.localizer;
        telemetry.addData("Raw ticks", "par0: %d, par1: %d, perp: %d",
                localizer.par0.getPositionAndVelocity().position,
                localizer.par1.getPositionAndVelocity().position,
                localizer.perp.getPositionAndVelocity().position);
    }


    /**
     * @return the difference between the robot's current angle and the Zero Angle
     */
    public double getAngleDifferenceFromZero() {
        double currentAngle = getZAngle(); // Get current angle
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

    /**
     *
     * @return the Z angle of the internal IMU in the control panel.
     */
    public double getZAngle() {
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
    }

    @Override
    public void periodic() {
   //     telemetry.addData("Camera Status", robot.sensors.camera.getLatestFreshDetections().get(0));
        // SUBSYSTEMS SHOULD PERIODICALLY POST THEIR DATA
        try {
            postEncoderData();
        } catch (Exception ignored) {}

        updatePoseEstimate();
    }

}


