package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Intake extends SubsystemBase {
    private Callisto robot;

    public CRServo spinServo;

    // Motor for the shoulder
    public DcMotorEx shoulderMotor;

    // Declare the extension servo
    public Servo extensionServo;


    public Intake(Callisto robot) {
        this.robot = robot;
        shoulderMotor = robot.hardwareMap.get(DcMotorEx.class, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        // Set up the servos
        extensionServo = robot.hardwareMap.get(Servo.class, Constants.EXTEND_INTAKE_SERVO);
        spinServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.SPIN_INTAKE_SERVO);
    }

    public boolean isExtended() {
        // TODO: guess; assuming 0 is extended skibidi big justice
        return extensionServo.getPosition() == 0;
    }

    public boolean isRetracted() {
        // TODO: guess; assuming 1 is retracted skibidi big justice
        return extensionServo.getPosition() == 1;
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



    public void setExtension(double position) {
        extensionServo.setPosition(position);
    }

    public void setSpinSpeed(double speed) {
        robot.telemetry.addData("Is it Spinning", speed != 0);
        spinServo.setPower(speed);
    }

    public void stopSpin() {
        spinServo.setPower(0.0);
    }

    // Method to get the current shoulder position
    public double getShoulderPosition() {
        return shoulderMotor.getCurrentPosition();
    }

    public void stopShoulder() {
        shoulderMotor.setPower(0);
    }

    @Override
    public void periodic() {

        String shoulderState = isUp() ? "Up"
                : isNearUp() ? "Near Up"
                : isDown() ? "Down"
                : isNearDown() ? "Near Down"
                : "Unknown";

        robot.telemetry.addData("Intake: ", "Extension: " + extensionServo.getPosition() + ", Shoulder: " + shoulderState);
        // output the current position of the shoulder motor
        robot.telemetry.addData("Shoulder Position", getShoulderPosition());
    }
}
