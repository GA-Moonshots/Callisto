package org.firstinspires.ftc.teamcode.commands;

import android.telecom.Call;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class MoveLift extends CommandBase {
    private Callisto robot;
    private Lift lift;
    private boolean controller = true;

    public MoveLift(Callisto callisto) {
        robot = callisto;
        this.lift = robot.lift;



        lift.motor1.setTargetPosition(250);

        addRequirements(lift);
    }

    @Override
    public void execute() {
        boolean isNegative = false;

        while (robot.opMode.opModeIsActive() && !lift.motor1.atTargetPosition()) {
            lift.motor1.set(isNegative ? -0.5 : 0.5);
            if(robot.player2.wasJustPressed(GamepadKeys.Button.B)) {
                lift.motor1.setTargetPosition(0);
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
