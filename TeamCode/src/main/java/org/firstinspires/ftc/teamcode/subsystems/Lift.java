 package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Lift extends SubsystemBase {

    private Callisto robot;
    private int currentPosition;
    private boolean isLiftActivated = false;

    // Enum to represent the different states of the basket (dump, nest, level)
    public enum BasketState {
        DUMP,
        NEST,
        LEVEL
    }
    // The current state of the basket
    private BasketState currentBasketState = BasketState.NEST;

    // MOTORS + SERVOS
    public MotorEx motor1;
    public Servo basket;

    // set up all the different motors
    public Lift(Callisto robot){
        robot = robot;
        basket = robot.hardwareMap.get(Servo.class, Constants.LIFT_BASKET_SERVO_NAME);
        motor1 = new MotorEx(robot.hardwareMap, Constants.LIFT_MOTOR_NAME);
        motor1.setInverted(true);
//        motor1.setRunMode(Motor.RunMode.VelocityControl);
        motor1.setRunMode(Motor.RunMode.PositionControl);
        motor1.setPositionCoefficient(0.17);
        motor1.setPositionTolerance(10);
        motor1.set(0);
    }

    public void nestBasket(){
        moveBasket(0.0);
        currentBasketState = BasketState.NEST;
    }

    public void levelBasket(){
        moveBasket(0.5);
        currentBasketState = BasketState.LEVEL;
    }


    public void dumpBasket(){
        moveBasket(1.0);
        currentBasketState = BasketState.DUMP;

    }





    /**
     *Moves the basket to a new location.
     *
     * <p>This method is intended for internal use within the basket subsystem.
     * It is not recommended for general use as higher-level shorthands,
     * such as those for dumping the basket, are typically more appropriate.
     *
     * @param pos The new location for the basket.
     */
    public void moveBasket(double pos){
        basket.setPosition(pos);
    }
}

