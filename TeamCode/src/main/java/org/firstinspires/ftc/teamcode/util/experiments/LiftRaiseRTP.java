package org.firstinspires.ftc.teamcode.util.experiments;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Callisto;

public class LiftRaiseRTP extends CommandBase {
    private final Callisto robot;
    private final LiftRTP lift;
    private final int targetPosition;
    private final double TIMEOUT = 12.0; // seconds
    private ElapsedTime timer;

    public LiftRaiseRTP(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.lift = robot.liftRTP;
        this.targetPosition = targetPosition;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        lift.motor1.setTargetPosition(targetPosition);
        lift.motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.motor1.setPower(1.0); // Use maximum power to raise
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Lift Raise RTP", "");
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Current Position", lift.motor1.getCurrentPosition());
        robot.telemetry.addData("Motor Power", lift.motor1.getPower());
    }

    @Override
    public boolean isFinished() {
        return !lift.motor1.isBusy() || timer.seconds() >= TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.setPower(0);
        lift.motor1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }
}
