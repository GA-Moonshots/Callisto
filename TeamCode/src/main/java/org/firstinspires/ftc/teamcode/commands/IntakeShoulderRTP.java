package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Constants;

public class IntakeShoulderRTP extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final int targetPosition;
    private final double TIMEOUT = 12.0; // seconds
    private ElapsedTime timer;

    public IntakeShoulderRTP(Callisto robot, int targetPosition) {
        this.robot = robot;
        this.intake = robot.intake;
        this.targetPosition = targetPosition;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        timer.reset();

        intake.shoulderMotor.setTargetPosition(targetPosition);
        intake.shoulderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidfCoefficients = intake.shoulderMotor.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        double pValue = pidfCoefficients.p;  // Extract the P value
        intake.shoulderMotor.setPositionPIDFCoefficients( pValue * 1);
        intake.shoulderMotor.setPower(0.85);
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Shoulder RTP", timer.seconds());
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Motor Power", intake.shoulderMotor.getPower());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
    }
}
