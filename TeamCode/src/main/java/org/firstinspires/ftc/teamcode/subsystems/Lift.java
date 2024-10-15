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
        motor1.setInverted(true);
//        motor1.setRunMode(Motor.RunMode.VelocityControl);
        motor1.setRunMode(Motor.RunMode.PositionControl);
        motor1.setPositionCoefficient(0.17);
        motor1.setPositionTolerance(10);
        motor1.set(0);
    }
}

