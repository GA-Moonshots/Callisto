package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

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
        double power = 0.35 * robot.player2.getRightY();
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
