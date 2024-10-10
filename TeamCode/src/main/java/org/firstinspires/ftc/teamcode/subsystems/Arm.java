package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

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
    private final MotorEx arm;
    private final ServoEx clawServo;

    // SERVOS
    //private final Servo clawServo;

    public Arm(Callisto callisto) {
        robot = callisto;
        telemetry = robot.telemetry;

        shoulderMotor = new MotorEx(robot.hardwareMap, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setVeloCoefficients(0.03, 0.4, 0.02);
        shoulderMotor.setRunMode(Motor.RunMode.VelocityControl);

        clawServo = new SimpleServo(robot.hardwareMap, Constants.CLAW_SERVO_NAME, 0, 180);

        arm = new MotorEx(robot.hardwareMap, Constants.ARM_MOTOR_NAME);
        shoulderMotor.setVeloCoefficients(0.2, 0.2, 0.02);
        arm.setRunMode(Motor.RunMode.VelocityControl);
    }

    public void moveShoulder(double angle) {
        // Angle negative to reverse it, 1/2 to slow it
        shoulderMotor.set(-angle/2);
    }

    public void moveArm(double amount) {
        arm.set(amount/2);
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
