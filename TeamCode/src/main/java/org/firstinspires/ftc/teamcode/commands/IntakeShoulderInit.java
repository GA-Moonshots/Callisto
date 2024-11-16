package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderInit extends CommandBase {
    private final Callisto robot;
    private final Intake intake;

    private final int targetPosition = 0; // Target position when shoulder is down
    private final double TIMEOUT = 3.0; // seconds
    private final Timing.Timer timer;

    public IntakeShoulderInit(Callisto robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.timer = new Timing.Timer((long) TIMEOUT); // Convert to milliseconds

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double power = 0.1;




        intake.shoulderMotor.set(power);
    }

    @Override
    public boolean isFinished() {
        return intake.isUp();
    }
}
