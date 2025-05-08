package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderRTP extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final int targetPosition;
    private final double TIMEOUT = 12.0; // seconds
    private ElapsedTime timer;
    double pValue;
    double iValue;
    double dValue;
    double fValue;

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
        pValue = pidfCoefficients.p * 0.65;
        iValue = pidfCoefficients.i;
        dValue = pidfCoefficients.d;
        fValue = pidfCoefficients.f;
        PIDFCoefficients tune = new PIDFCoefficients(pValue, iValue, dValue, fValue);
        intake.shoulderMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION , tune);
        intake.shoulderMotor.setTargetPositionTolerance(100);
        intake.shoulderMotor.setPower(0.45);
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Shoulder RTP", timer.seconds());
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Pid",pValue);
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() > TIMEOUT;
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopShoulder();
    }
}
