package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Lift extends SubsystemBase {
    // Instance variables
    private Callisto robot;

    // Enum to represent the different states of the basket (dump, nest, level)
    public enum BasketState {
        DUMP,
        NEST,
        LEVEL
    }

    // The current state of the basket
    public BasketState currentBasketState = BasketState.NEST;

    // Motors and Servos
    public DcMotorEx motor1;
    public Servo basket;

    // Constructor
    public Lift(Callisto robot) {
        this.robot = robot;
        basket = robot.hardwareMap.get(Servo.class, Constants.LIFT_BASKET_SERVO_NAME);

        // Initialize motor1 as DcMotorEx to use RUN_TO_POSITION
        motor1 = robot.hardwareMap.get(DcMotorEx.class, Constants.LIFT_MOTOR_NAME);
        motor1.setDirection(DcMotorEx.Direction.FORWARD);
        motor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor1.setTargetPosition(0);
        motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor1.setPower(0);
    }

    /**
     * Nests the basket.
     */
    public void nestBasket() {
        moveBasket(0.0);
        currentBasketState = BasketState.NEST;
    }

    /**
     * Levels the basket.
     */
    public void levelBasket() {
        moveBasket(0.4);
        currentBasketState = BasketState.LEVEL;
    }

    /**
     * Dumps the contents of the basket.
     */
    public void dumpBasket() {
        moveBasket(1.0);
        currentBasketState = BasketState.DUMP;
    }

    /**
     * Moves the basket to a specified position.
     *
     * @param pos The new position for the basket servo.
     */
    public void moveBasket(double pos) {
        basket.setPosition(pos);
    }

    public boolean isUp() {
        return Math.abs(motor1.getCurrentPosition() - Constants.HIGH_HEIGHT) < 230;
    }

    public boolean isDown() { return motor1.getCurrentPosition() <= Constants.LOW_HEIGHT;}

    @Override
    public void periodic() {
        // Telemetry for debugging
        robot.telemetry.addData("Lift Position", motor1.getCurrentPosition());
    }
}
