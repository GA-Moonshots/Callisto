
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.experiments.IntakeOld;

import java.util.concurrent.TimeUnit;

public class IntakeShoulderUp extends CommandBase {


    private final Callisto robot;
    private final Intake intake;
     // acceptable error in ticks
    private final double TIMEOUT = 3.0; // seconds
    protected Timing.Timer timer;

    public IntakeShoulderUp(Callisto robot) {
        this.robot = robot;
        this.intake = this.robot.intake;


        timer = new Timing.Timer((long) TIMEOUT * 1000, TimeUnit.MILLISECONDS);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double power;

        if (timer.elapsedTime() < 1500) {
            intake.shoulderMotor.set(0.75); // First 1500ms
        } else if (timer.elapsedTime() < 2000 && timer.elapsedTime() >= 1500) {
            intake.shoulderMotor.set(0.2); // Next 500ms (1500-2000ms)
        } else {
            intake.shoulderMotor.set(0);
        }


    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.stopMotor();
    }
}