package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.Timing;

@Config
public class LiftRaise extends CommandBase {
    // PID constants moved here
    public static double kP = 0.85;
    public static double kI = 0.0055;
    public static double kD = 0.03;

    private final Callisto robot;
    private final Lift lift;
    private final PIDController pid;
    private final int targetPosition;
    private final double TOLERANCE = 5; // acceptable error in ticks
    private final double TIMEOUT = 12.0; // seconds
    protected Timing.Timer timer;

    public LiftRaise(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        pid = new PIDController(kP, kI, kD);  // Updated reference
        pid.setSetPoint(targetPosition);
        pid.setTolerance(TOLERANCE);

        timer = new Timing.Timer((long) TIMEOUT);
        lift.motor1.setTargetPosition(targetPosition);

        addRequirements(lift);
    }

    @Override
    public void initialize(){
        timer.start();
        lift.motor1.setRunMode(Motor.RunMode.PositionControl);
    }

    @Override
    public void execute() {
        double currentPosition = lift.motor1.getCurrentPosition();
        double power = pid.calculate(currentPosition);

        lift.motor1.set(power);

        // Telemetry for debugging
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Current Position", currentPosition);
        robot.telemetry.addData("Motor Power", power);
    }

    @Override
    public boolean isFinished() {
        return timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.stopMotor();
    }
}
