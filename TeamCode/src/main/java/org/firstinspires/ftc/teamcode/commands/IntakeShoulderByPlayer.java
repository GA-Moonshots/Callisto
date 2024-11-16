package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.IntakeExperimental;


public class IntakeShoulderByPlayer extends CommandBase {

    private final Callisto robot;
    private final IntakeExperimental intake;

    public IntakeShoulderByPlayer (Callisto robot){
        this.robot = robot;
        this.intake = this.robot.intakeExperimental;


        addRequirements(robot.intakeExperimental);
    }

    @Override
    public void execute() {

        robot.telemetry.addData("is the Command Running", true);
        double power = 0.35 * robot.player2.getRightY();
        // Deadzone
        if(Math.abs(power) < 0.05){
            power = 0;
        }

        intake.shoulderMotor.set(power);
    }

    public boolean isFinished() {
        return false;
    }
}
