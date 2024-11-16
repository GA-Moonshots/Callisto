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
        this.robot = robot;
        basket = robot.hardwareMap.get(Servo.class, Constants.LIFT_BASKET_SERVO_NAME);
        motor1 = new MotorEx(robot.hardwareMap, Constants.LIFT_MOTOR_NAME);
        motor1.resetEncoder();
        motor1.setInverted(false);
        motor1.setRunMode(Motor.RunMode.PositionControl);
        motor1.setPositionCoefficient(0.2);
        motor1.setPositionTolerance(5);
        motor1.set(0);
    }

    /**
     * Levels the basket.
     *
     * This method moves the basket to a level position by calling {@link #moveBasket(double)} with a
     * value of 0.0. It then updates the {@link #currentBasketState} to {@link BasketState#LEVEL}.
     */
    public void nestBasket(){
        moveBasket(0.0);
        currentBasketState = BasketState.NEST;
    }

    public void levelBasket(){
        moveBasket(0.4);
        currentBasketState = BasketState.LEVEL;
    }

    /**
     * Dumps the contents of the basket.*
     * This method fully tilts the basket by calling {@link #moveBasket(double)} with a
     * value of 1.0. It then updates the {@link #currentBasketState} to {@link BasketState#DUMP}.
     */
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

    public boolean isUp() {
        return currentPosition >= Constants.MID_HEIGHT;
    }

    @Override
    public void periodic() {
        // EVERY SUBSYSTEM MUST HAVE A PERIODIC FUNCTION TO CONCISELY OUTPUT STATUS
        robot.telemetry.addData("Lift Position", robot.lift.motor1.getCurrentPosition());
    }
}

