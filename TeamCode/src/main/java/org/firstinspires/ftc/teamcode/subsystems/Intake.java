package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Intake extends SubsystemBase {
    private Callisto robot;

    // Continuous Rotation Servos
    private com.qualcomm.robotcore.hardware.CRServo spinServo;
    public com.qualcomm.robotcore.hardware.CRServo extendServo;

    // Motor for the shoulder
    public MotorEx shoulderMotor;

    // Declare the limit switches
    public TouchSensor nearSwitch; // Near the robot
    public TouchSensor farSwitch;


    public Intake(Callisto robot) {
        this.robot = robot;
        shoulderMotor = new MotorEx(robot.hardwareMap, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setRunMode(Motor.RunMode.RawPower);
        shoulderMotor.resetEncoder();

        // Set up the servos
        extendServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.EXTEND_INTAKE_SERVO);
        spinServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.SPIN_INTAKE_SERVO);

        // Set up the sensors
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

    public boolean isBetween() {
        return !isExtended() && !isRetracted();
    }

    public boolean isUp() {
        return shoulderMotor.getCurrentPosition() > - 50;

    }

    public boolean isNearUp() {
        return shoulderMotor.getCurrentPosition() > - 100 ;
    }

    public boolean isNearDown() {
        return shoulderMotor.getCurrentPosition() < - 275;
    }

    public boolean isDown() {
        return shoulderMotor.getCurrentPosition() < - 325;
    }



    public void setExtensionSpeed(double speed) {
        robot.telemetry.addData("Is it Extending", speed != 0);
        extendServo.setPower(speed);
    }

    public void setSpinSpeed(double speed) {
        robot.telemetry.addData("Is it Spinning", speed != 0);
        spinServo.setPower(speed);
    }

    public void stopExtension() {
        extendServo.setPower(0.0); // Stops the servo
    }

    public void stopSpin() {
        spinServo.setPower(0.0);
    }

    // Method to get the current shoulder position
    public double getShoulderPosition() {
        return shoulderMotor.getCurrentPosition();
    }

    public void stopShoulder() {
        shoulderMotor.stopMotor();
    }

    @Override
    public void periodic() {
        String state = isExtended() ? "Extended"
                : isRetracted() ? "Retracted"
                : isBetween() ? "Moving"
                : "Unknown";

        String shoulderState = isUp() ? "Up"
                : isNearUp() ? "Near Up"
                : isDown() ? "Down"
                : isNearDown() ? "Near Down"
                : "Unknown";

        robot.telemetry.addData("Intake: ", "Extension: " + state + ", Shoulder: " + shoulderState);
        // output the current position of the shoulder motor
        robot.telemetry.addData("Shoulder Position", getShoulderPosition());
    }
}
