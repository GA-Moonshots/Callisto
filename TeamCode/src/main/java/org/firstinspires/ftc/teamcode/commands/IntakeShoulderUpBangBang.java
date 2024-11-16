package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderUpBangBang extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final int firstThreshold = 700; // First magic number
    private final int targetPosition = 1100; // Final target height
    private final double TOLERANCE = 5; // Acceptable error in ticks
    private final double TIMEOUT = 12.0; // Seconds
    protected Timing.Timer timer;

    public IntakeShoulderUpBangBang(Callisto robot) {
        this.robot = robot;
        this.intake = this.robot.intake;
        timer = new Timing.Timer((long) TIMEOUT);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double currentPosition = intake.shoulderMotor.getCurrentPosition();
        double power;

        if (currentPosition < firstThreshold) {
            // Lift up at 0.75 speed
            power = 0.75;
        } else if (currentPosition < targetPosition) {
            // Reverse at -0.25 to brake
            power = -0.25;
        } else {
            // Stop the motor once target position is reached
            power = 0;
        }

        intake.shoulderMotor.set(power);
        robot.telemetry.addData("Shoulder Position:", currentPosition);
        robot.telemetry.addData("Motor Power:", power);
    }

    @Override
    public boolean isFinished() {
        double currentPosition = intake.shoulderMotor.getCurrentPosition();
        return currentPosition >= targetPosition || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.stopMotor();
        intake.isShoulderUp = !intake.isShoulderUp;
    }
}
