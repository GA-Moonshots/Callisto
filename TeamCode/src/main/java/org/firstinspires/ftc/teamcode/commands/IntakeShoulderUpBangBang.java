package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderUpBangBang extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final double TIMEOUT = 12.0; // Timeout in seconds
    private final Timing.Timer timer;

    public IntakeShoulderUpBangBang(Callisto robot) {
        this.robot = robot;
        this.intake = this.robot.intake;
        this.timer = new Timing.Timer((long) TIMEOUT); // Convert to milliseconds

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("Intake Shoulder Up Bang Bang", "Running");
        double power = 0.0;

        if (intake.isNearDown()) {
            // Apply upward power
            power = 0.75;
        } else if (intake.isNearUp()) {
            // Apply reverse power to slow down
            power = -0.25;
        }

        intake.shoulderMotor.set(power);
    }

    @Override
    public boolean isFinished() {
        return intake.isUp() || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
    }
}
