package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

import java.util.concurrent.TimeUnit;

public class IntakeExtensionWithTimeout extends CommandBase {
    private Callisto robot;
    private Intake intake;

    private Timing.Timer timer;

    private int position;

    public IntakeExtensionWithTimeout(Callisto robot, int position, long timeoutMilliseconds) {
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
        return timer.done() || (intake.extensionServo.getPosition() == position);
    }
}
