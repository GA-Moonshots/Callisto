package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Arm extends SubsystemBase {
    // INSTANCE VARIABLES
    private final Callisto robot;
    private final Telemetry telemetry;
    private boolean isClawOpen = true;

    // MOTORS
    public final MotorEx shoulderMotor;

    // SERVOS
    //private final Servo clawServo;

    public Arm(Callisto callisto) {
        robot = callisto;
        telemetry = robot.telemetry;

        shoulderMotor = new MotorEx(robot.hardwareMap, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setVeloCoefficients(0.2, 0.0, 0.05);
        shoulderMotor.setRunMode(Motor.RunMode.VelocityControl);

        //clawServo = new Servo(robot.hardwareMap, Constants.CLAW_SERVO_NAME);
    }

    public void moveArm(double angle) {
        shoulderMotor.set(angle);
    }

    public void toggleClaw() {
        isClawOpen = !isClawOpen;

        if (isClawOpen) {

        }
        else {

        }
    }
}
