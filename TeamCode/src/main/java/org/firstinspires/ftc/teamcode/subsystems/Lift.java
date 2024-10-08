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
    private MotorEx motor1;
    private MotorEx motor2;

// set up all the different motors
    public Lift(Callisto callisto){
        motor1 = new MotorEx(robot.hardwareMap, "liftRight");
        motor1.setRunMode(Motor.RunMode.VelocityControl);
        motor1.setPositionCoefficient(0.5);
        motor1.setPositionTolerance(3.6);


        motor2 = new MotorEx(robot.hardwareMap,"liftRight");
        robot = callisto;

    }

    public void togglePosition() {
        isLiftActivated = !isLiftActivated;

        if (isLiftActivated) {
            up();
        } else {
            down();
        }
    }

    public void up(){
        motor1.setTargetPosition(100);
        motor1.set(0);

        while(!motor1.atTargetPosition()){
            motor1.set(0.75);
        }

        currentPosition = motor1.getCurrentPosition();
        motor1.stopMotor(); // stop the motor
    }

    public void down(){
        motor1.setTargetPosition(0);

        while(!motor1.atTargetPosition()){
            motor1.set(-0.75);
        }

        currentPosition = motor1.getCurrentPosition();
        motor1.stopMotor(); // stop the motor
    }


}

