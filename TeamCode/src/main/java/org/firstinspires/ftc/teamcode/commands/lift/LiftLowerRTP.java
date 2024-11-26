package org.firstinspires.ftc.teamcode.commands.lift;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Constants;

public class LiftLowerRTP extends CommandBase {
    private final Callisto robot;
    private final Lift lift;
    private final int targetPosition = 0; // Assuming 0 is the lowest position
    private double timeout = 1.0; // seconds
    private ElapsedTime timer;

    public LiftLowerRTP(Callisto robot) {
        this.robot = robot;
        this.lift = robot.lift;

        addRequirements(lift);
    }

    @Override
    public void initialize() {
        if (lift.currentBasketState != Lift.BasketState.LEVEL) {
            lift.levelBasket();
        }
        if(lift.isUp()) timeout++;

        timer = new ElapsedTime();
        timer.reset();

        lift.motor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // remove breaking behavior so gravity can assist
        lift.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        lift.motor1.setPower(-0.63);
    }

    @Override
    public void execute() {
        // Telemetry for debugging
        robot.telemetry.addData("Lift Lower RTP", timer.seconds());
        robot.telemetry.addData("Target Position", targetPosition);
        robot.telemetry.addData("Motor Power", lift.motor1.getPower());

        if(lift.motor1.getCurrentPosition() >= Constants.MID_HEIGHT){
            lift.motor1.setPower(-0.80);
        }else if(lift.motor1.getCurrentPosition() < Constants.MID_HEIGHT){
            lift.motor1.setPower(-0.40);
        }
    }

    @Override
    public boolean isFinished() {
        return timer.seconds() >= timeout;
    }

    @Override
    public void end(boolean interrupted) {
        lift.motor1.setPower(0);
        lift.motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (!interrupted) {
            lift.motor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
