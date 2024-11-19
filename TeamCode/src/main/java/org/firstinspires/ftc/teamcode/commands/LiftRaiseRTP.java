package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftRaiseRTP extends CommandBase {
    private final Callisto robot;
    private final Lift lift;
    private final int targetPosition;
    private final double TIMEOUT = 12.0; // seconds
    private ElapsedTime timer;

    public LiftRaiseRTP(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.lift = robot.lift;
        this.targetPosition = targetPosition;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        lift.motor1.setTargetPosition(targetPosition);
        lift.motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidfCoefficients = lift.motor1.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        double pValue = pidfCoefficients.p;  // Extract the P value
        lift.motor1.setPositionPIDFCoefficients( pValue * 0.6);
        lift.motor1.setPower(0.85);
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Lift Raise RTP", timer.seconds());
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Motor Power", lift.motor1.getPower());

        if(lift.motor1.getCurrentPosition() > Constants.HIGH_HEIGHT * 0.9) {
            lift.motor1.setPower(0);
        }else{
            lift.motor1.setPower(0.85);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.setPower(0);
    }
}
