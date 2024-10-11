 package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Lift extends SubsystemBase {

    private Callisto robot;
    private int currentPosition;
    private boolean isLiftActivated = false;

    //motors
    public MotorEx motor1;

    // set up all the different motors
    public Lift(Callisto callisto){
        robot = callisto;
        motor1 = new MotorEx(robot.hardwareMap, Constants.LIFT_MOTOR_NAME);
        motor1.encoder.setDirection(Motor.Direction.REVERSE);
//        motor1.setRunMode(Motor.RunMode.VelocityControl);
        motor1.setRunMode(Motor.RunMode.PositionControl);
        motor1.setPositionCoefficient(0.1);
        motor1.set(0);
        motor1.setPositionTolerance(10);
    }

    public void togglePosition() {
        isLiftActivated = !isLiftActivated;

        moveToPosition(-100);
    }

    // For the lift to go up, the value is negative
    // the Max value is -580
    private void moveToPosition(int targetPosition) {
        motor1.setTargetPosition(targetPosition);

        robot.telemetry.addData("Oh No", motor1.getCurrentPosition());
        robot.telemetry.update();

        motor1.set(targetPosition);

        // Non-blocking movement
//        while (!motor1.atTargetPosition()) {
//            motor1.set(targetPosition > motor1.getCurrentPosition() ? 0.75 : -0.75);
//        }

        motor1.stopMotor();
    }

    public int getCurrentPosition() {
        return motor1.getCurrentPosition();
    }


}

