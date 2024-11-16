package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderToggle extends CommandBase {
    private final Intake intake;
    private final Callisto robot;
    private CommandBase shoulderCommand;


    public IntakeShoulderToggle(Callisto robot) {
        this.robot = robot;
        this.intake = robot.intake;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        if (intake.isUp()) {
            // Shoulder is up, move it down
            shoulderCommand = new IntakeShoulderDown(robot);
        } else {
            // Shoulder is down, move it up
            shoulderCommand = new IntakeShoulderUpBangBang(robot);
        }
    }

    @Override
    public void execute() {
        shoulderCommand.schedule();
    }

    @Override
    public boolean isFinished() {
        return shoulderCommand.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        shoulderCommand.end(interrupted);
    }
}
