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
        this.lift.motor1.setTargetPosition(-targetPosition);

        addRequirements(lift);
    }

    @Override
    public void execute() {
        lift.motor1.set(-0.75);

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
