package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderDown extends CommandBase {
    private final Callisto robot;
    private final Intake intake;

    private final int targetPosition = 0; // Target position when shoulder is down
    private final double TIMEOUT = 3.0; // seconds
    private final Timing.Timer timer;

    public IntakeShoulderDown(Callisto robot) {
        this.robot = robot;
        this.intake = robot.intake;
        this.timer = new Timing.Timer((long) (TIMEOUT * 1000)); // Convert to milliseconds

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("Intake Shoulder Down", "Running");
        double power = 0.0;

        if (!intake.isUp()) { // Add a small buffer
            // Apply a small upward power to control descent
            power = -0.15; // Adjust this value as needed to prevent slamming
        }
        else if (intake.isNearDown()) {
            //Apply break
            power = 0.05;
        }


        intake.shoulderMotor.set(power);
    }

    @Override
    public boolean isFinished() {
        return intake.isDown() || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
    }
}
