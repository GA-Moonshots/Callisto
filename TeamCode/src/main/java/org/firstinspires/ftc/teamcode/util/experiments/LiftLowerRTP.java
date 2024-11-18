package org.firstinspires.ftc.teamcode.util.experiments;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Callisto;

public class LiftLowerRTP extends CommandBase {
    private final Callisto robot;
    private final LiftRTP lift;
    private final int targetPosition = 0; // Assuming 0 is the lowest position
    private final double TIMEOUT = 1.5; // seconds
    private ElapsedTime timer;

    public LiftLowerRTP(Callisto robot) {
        this.robot = robot;
        this.lift = null;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (Math.abs(lift.basket.getPosition() - 0.4) > 0.1) {
            lift.levelBasket();
        }

        timer = new ElapsedTime();
        timer.reset();

        lift.motor1.setTargetPosition(targetPosition);
        lift.motor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift.motor1.setPower(0.5); // Use lower power since gravity assists
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Lift Lower RTP", "");
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

        if (!interrupted) {
            lift.motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
