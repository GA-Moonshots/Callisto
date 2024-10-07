package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
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
    private final MotorEx shoulderMotor;
    private final ServoEx clawServo;

    // SERVOS
    //private final Servo clawServo;

    public Arm(Callisto callisto) {
        robot = callisto;
        telemetry = robot.telemetry;

        shoulderMotor = new MotorEx(robot.hardwareMap, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setVeloCoefficients(0.2, 0.0, 0.05);
        shoulderMotor.setRunMode(Motor.RunMode.VelocityControl);

        clawServo = new SimpleServo(robot.hardwareMap, Constants.CLAW_SERVO_NAME, 0, 180);
    }

    public void moveArm(double angle) {
        shoulderMotor.set(angle);
    }

    public void toggleClaw() {
        isClawOpen = !isClawOpen;

        if (isClawOpen) {
            clawServo.turnToAngle(180);
        }
        else {
            clawServo.turnToAngle(0);
        }
    }
}
