package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class RaiseLift extends CommandBase {
    // REFERENCES
    private Callisto robot;
    private Lift lift;
    private final GamepadEx player2;
    // ASSETS
    private boolean controller = true;
    private boolean isNegative = false;
    private int targetPosition;


    public RaiseLift(Callisto robot, int targetPosition) {
        //TODO: Auto timeout this command
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        this.player2 = robot.player2;

        lift.motor1.setTargetPosition(targetPosition);

        addRequirements(lift);
    }

    @Override
    public void execute() {
//        // Set the target position based on button press
//        if (player2.gamepad.dpad_down) {
//            targetPosition = Constants.LOW_HEIGHT;
//        } else if (player2.gamepad.dpad_left) {
//            targetPosition = Constants.MID_HEIGHT;
//        } else if (player2.gamepad.dpad_up) {
//            targetPosition = Constants.HIGH_HEIGHT;
//        }

        robot.telemetry.addData("Target Position", targetPosition);

        // TODO: remove this update. We only want to update the telemetry once (in the SensorPackage)
        robot.telemetry.update();
        // Set the motor to the target position
        lift.motor1.setTargetPosition(targetPosition);

        robot.telemetry.addData("Position", lift.motor1.getCurrentPosition());

        // Move towards the target position
        if (!lift.motor1.atTargetPosition()) {
            lift.motor1.set(isNegative ? -0.5 : 0.5);
            if (lift.motor1.getCurrentPosition() == 0) {
                isNegative = true;
            }
        }
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        // TODO: include a timeout so we don't burn out the motor
        return false;
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        lift.motor1.stopMotor();
    }
}
