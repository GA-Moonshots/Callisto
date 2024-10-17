package org.firstinspires.ftc.teamcode.commands;

import android.telecom.Call;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Constants;

public class MoveLift extends CommandBase {
    private Callisto robot;
    private Lift lift;
    private final GamepadEx player2;
    private boolean controller = true;
    private boolean isNegative = false;
    private int targetPosition;


    public MoveLift(Callisto callisto) {
        robot = callisto;
        this.lift = robot.lift;

        player2 = callisto.player2;



        lift.motor1.setTargetPosition(250);

        addRequirements(lift);
    }

    @Override
    public void execute() {
        // Set the target position based on button press
        if (player2.gamepad.dpad_down) {
            targetPosition = Constants.LOW_HEIGHT;
        } else if (player2.gamepad.dpad_left) {
            targetPosition = Constants.MID_HEIGHT;
        } else if (player2.gamepad.dpad_up) {
            targetPosition = Constants.HIGH_HEIGHT;
        }

        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.update();
        // Set the motor to the target position
        lift.motor1.setTargetPosition(targetPosition);

        // Move towards the target position
        if (!lift.motor1.atTargetPosition()) {
            lift.motor1.set(isNegative ? -0.5 : 0.5);
            if (targetPosition == 0) {
                isNegative = true;
            }
        }
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return false;
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        lift.motor1.stopMotor();
    }
}
