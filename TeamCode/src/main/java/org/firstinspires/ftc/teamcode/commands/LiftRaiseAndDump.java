package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;

@Config
public class LiftRaiseAndDump extends CommandBase {
    // PID constants moved here
    public static double kP = 0.8;
    public static double kI = 0.005;
    public static double kD = 0.01;

    private final Callisto robot;
    private final Lift lift;
    private final PIDController pid;
    private final int targetPosition;
    private final double TOLERANCE = 8; // acceptable error in ticks
    private final double DEFAULT_TIMEOUT = 12.0; // seconds
    protected Timing.Timer timer;
    private boolean isFinished;

    public LiftRaiseAndDump(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        pid = new PIDController(kP, kI, kD);  // Updated reference
        pid.setSetPoint(targetPosition);
        pid.setTolerance(TOLERANCE);

        timer = new Timing.Timer((long) DEFAULT_TIMEOUT);
        lift.motor1.setTargetPosition(targetPosition);

        addRequirements(lift);
    }

    public LiftRaiseAndDump(Callisto robot, int targetPosition, double timeout) {
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;
        pid = new PIDController(kP, kI, kD);  // Updated reference
        pid.setSetPoint(targetPosition);
        pid.setTolerance(TOLERANCE);

        timer = new Timing.Timer((long) timeout);
        lift.motor1.setTargetPosition(targetPosition);

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        double currentPosition = lift.motor1.getCurrentPosition();
        double power = pid.calculate(currentPosition);
        power = Math.max(Math.min(power, 0.75), -0.75); // constrain power between -0.75 and 0.75

        lift.motor1.set(power);



        if (pid.atSetPoint()) {
            lift.dumpBasket();
            long dumpTime = timer.elapsedTime();
            if(dumpTime + 2 < timer.elapsedTime()){
                robot.telemetry.addData("Is levelling???", true);
                lift.levelBasket();
                isFinished = true;
            }

        }

        // Telemetry for debugging
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Current Position", currentPosition);
        robot.telemetry.addData("Motor Power", power);
    }

    @Override
    public boolean isFinished() {
        return isFinished || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.stopMotor();
    }
}
