package org.firstinspires.ftc.teamcode.commands.intake;

import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;


public class IntakeShoulderByPlayer extends CommandBase {

    private final Callisto robot;
    private final Intake intake;

    public IntakeShoulderByPlayer (Callisto robot){
        this.robot = robot;
        this.intake = this.robot.intake;


        addRequirements(robot.intake);
    }

    @Override
    public void execute() {

        robot.telemetry.addData("is the Intake shoulder by player Command Running", true);
        //double power = 0.45 * -robot.player2.getRightY();
        double power = 0.45 * -(Math.pow((robot.player2.getRightY()), 3));
        // Dead zone
        if(Math.abs(power) < 0.05){
            power = 0;
        }

        intake.shoulderMotor.setPower(power);
    }

    public boolean isFinished() {
        return false;
    }
}
