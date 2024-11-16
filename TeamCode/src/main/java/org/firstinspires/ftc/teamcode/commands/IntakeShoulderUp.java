
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderUp extends CommandBase {
    // PID constants moved here
    private static double kP = 0.4;
    private static double kI = 0.005;
    private static double kD = 0.01;

    private final Callisto robot;
    private final Intake intake;
    private final PIDController pid;
    private final int targetPosition;
    private final double TOLERANCE = 5; // acceptable error in ticks
    private final double TIMEOUT = 12.0; // seconds
    protected Timing.Timer timer;

    public IntakeShoulderUp(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.intake = this.robot.intake;
        this.targetPosition = targetPosition;
        pid = new PIDController(kP, kI, kD);  // Updated reference
        pid.setSetPoint(targetPosition);
        pid.setTolerance(TOLERANCE);

        timer = new Timing.Timer((long) TIMEOUT);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("Shoulder:", intake.shoulderMotor.getCurrentPosition());
        double currentPosition = intake.shoulderMotor.getCurrentPosition();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, 0), 0.8);

        intake.shoulderMotor.set(power);
    }

    @Override
    public boolean isFinished() {
        return pid.atSetPoint() || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.stopMotor();
        intake.isShoulderUp = !intake.isShoulderUp;
    }
}