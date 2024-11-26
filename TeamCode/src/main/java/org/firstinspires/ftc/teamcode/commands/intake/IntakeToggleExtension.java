package org.firstinspires.ftc.teamcode.commands.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeToggleExtension extends CommandBase {

    private final Callisto robot;
    private Intake intake;

    public IntakeToggleExtension(Callisto robot) {
        this.robot = robot;
        this.intake = robot.intake;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.extensionServo.setPosition(intake.isExtended ? 1 : 0);
    }

    @Override
    public boolean isFinished() {
        if (intake.isExtended)
            return intake.extensionServo.getPosition() == 0;
        else
            return intake.extensionServo.getPosition() == 1;
    }

    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.setPower(0.0);  //set(0);
        intake.isExtended = !intake.isExtended;
    }
}
