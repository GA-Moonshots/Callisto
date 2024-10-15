package org.firstinspires.ftc.teamcode.commands;

import android.telecom.Call;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

public class MoveLift extends CommandBase {
    private Callisto robot;
    private Lift lift;

    public MoveLift(Callisto callisto, int targetPosition) {
        robot = callisto;
        this.lift = robot.lift;
        lift.motor1.setTargetPosition(250);

        addRequirements(lift);
    }

    @Override
    public void execute() {
        int i = 0;
        while (robot.opMode.opModeIsActive() && !lift.motor1.atTargetPosition()) {
            robot.telemetry.addData("is in while:", ++i);
            robot.telemetry.update();
            lift.motor1.set(0.5);
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
