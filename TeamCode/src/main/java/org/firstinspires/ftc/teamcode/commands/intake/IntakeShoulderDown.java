
package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeShoulderDown extends CommandBase {


    private final Callisto robot;
    private final Intake intake;
    // acceptable error in ticks
    private final double TIMEOUT = 2.0; // seconds
    protected Timing.Timer timer;

    public IntakeShoulderDown(Callisto robot) {
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

        if (timer.elapsedTime() < 800) {
            intake.shoulderMotor.setPower(-0.40); // First 500ms
        } else if (timer.elapsedTime() < 1250 && timer.elapsedTime() >= 800) {
            intake.shoulderMotor.setPower(0.2); // Next 500ms (500-1000ms)
        } else {
            intake.shoulderMotor.setPower(0); // Remaining time (1000-2000ms)
        }


    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
    }
}