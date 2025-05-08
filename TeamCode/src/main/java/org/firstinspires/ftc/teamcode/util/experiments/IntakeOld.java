package org.firstinspires.ftc.teamcode.util.experiments;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeOld extends SubsystemBase {

    private Callisto robot;

    // Constant Rotation SERVOS
    private com.qualcomm.robotcore.hardware.CRServo spinServo;
    public com.qualcomm.robotcore.hardware.CRServo extendServo;

    //MOTOR for the shoulder
    public MotorEx shoulderMotor;

    // Declare the limit switch and  states
    public TouchSensor nearSwitch; // as in near to the robot
    public TouchSensor farSwitch;

    public boolean isShoulderUp = true;

    public IntakeOld(Callisto robot){
        this.robot = robot;
        shoulderMotor = new MotorEx(robot.hardwareMap, Constants.SHOULDER_MOTOR_NAME);
        // set up the servos
        extendServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.EXTEND_INTAKE_SERVO);
        spinServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.SPIN_INTAKE_SERVO);
        // set up the sensors
        nearSwitch = robot.hardwareMap.get(TouchSensor.class, Constants.NEAR_SENSOR);
        farSwitch = robot.hardwareMap.get(TouchSensor.class, Constants.FAR_SENSOR);
    }

    public boolean isTriggered(TouchSensor touchSensor) {
        return touchSensor.isPressed();
    }

    public boolean isExtended() {
        return isTriggered(farSwitch);
    }

    public boolean isRetracted() {
        return isTriggered(nearSwitch);
    }

    public boolean isMoving() {
        return !isTriggered(farSwitch) && !isTriggered(nearSwitch);
    }

    public void setExtensionSpeed(double speed) {
        robot.telemetry.addData("Is it Extending", true);
        extendServo.setPower(speed);
    }

    public void setSpinSpeed(double speed) {
        robot.telemetry.addData("Is it Spinning ", true);
        spinServo.setPower(speed);
    }

    public void stopExtension() {
        extendServo.setPower(0.0); // Stops the servo
    }

    public void stopSpin() { spinServo.setPower(0.0); }

    @Override
    public void periodic() {
        robot.telemetry.addData("Shoulder Position", shoulderMotor.getCurrentPosition());
        String state = isExtended() ? "Extended"
                : isRetracted() ? "Retracted"
                : isMoving() ? "Moving"
                : "Unknown";

        robot.telemetry.addData("Intake State", state);
    }
}