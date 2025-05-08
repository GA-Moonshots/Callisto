package org.firstinspires.ftc.teamcode.util.experiments;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.controller.PIDController;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderByPID extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final int targetPosition;
    private final double TIMEOUT = 12.0; // seconds
    private ElapsedTime timer;
    private PIDController pidController;

    // PID coefficients (these values will need to be tuned)
    private static final double kP = 0.1;
    private static final double kI = 0.05;
    private static final double kD = 0.0;

    public IntakeShoulderByPID(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.intake = robot.intake;
        this.targetPosition = targetPosition;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

//        // Initialize the PID Controller
//
//        pidController = new PIDController(kP, kI, kD);
//        pidController.setSetPoint(targetPosition);
//
//        // Optionally set tolerance (e.g., 10 ticks)
//        pidController.setTolerance(5);
    }

    @Override
    public void execute() {
        // Get the current position of the motor
        double currentPosition = intake.getShoulderPosition();
//
//        // Calculate the power using the PID controller
//        double power = pidController.calculate(currentPosition);
//
//        // Clamp the power to avoid exceeding motor limits
//        power = Math.max(-0.8, Math.min(0.8, power));
//
//        // Apply the power to the motor
//        intake.shoulderMotor.set(power);



    }

    @Override
    public boolean isFinished() {
        // Check if the PID controller is within the tolerance or if timeout has elapsed
        return pidController.atSetPoint() || timer.seconds() > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
//        intake.shoulderMotor.resetEncoder();
//        intake.stopShoulder();
    }
}
