package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeRetract extends CommandBase {
    private final Callisto robot;
    private final Intake intake;

    protected Timing.Timer timer;
    private final double TIMEOUT = 3.0;

    public IntakeRetract(Callisto robot) {
        this.robot = robot;
        intake = this.robot.intake;
        timer = new Timing.Timer((long) TIMEOUT);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("Retract countdown:", timer.remainingTime());
        intake.setExtensionSpeed(-0.95);
    }

    @Override
    public boolean isFinished() {
        return intake.nearSwitch.isPressed() || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopExtension();
    }
}
