package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeExtensionWithTimeout extends CommandBase {
    private Callisto robot;
    private Intake intake;

    private Timing.Timer timer;

    private double position;

    public IntakeExtensionWithTimeout(Callisto robot, double position, long timeoutMilliseconds) {
        this.robot = robot;
        intake = this.robot.intake;

        this.position = position;

        timer = new Timing.Timer(timeoutMilliseconds, TimeUnit.MILLISECONDS);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        intake.setExtension(position);
    }

    @Override
    public boolean isFinished() {
        return timer.done();// || (Math.abs(intake.extensionServo.getPosition() - position) < 0.05);
    }
}
