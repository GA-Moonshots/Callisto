package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeExtend extends CommandBase {
    private final Callisto robot;
    private final Intake intake;
    private final GamepadEx player2;

    protected Timing.Timer timer;
    private final double TIMEOUT = 3.0;

    public IntakeExtend(Callisto robot) {
        this.robot = robot;
        intake = this.robot.intake;

        player2 = robot.player2;

        timer = new Timing.Timer((long) TIMEOUT);

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        intake.setExtensionSpeed(.95);
    }

    @Override
    public boolean isFinished() {
        return  intake.farSwitch.isPressed() || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        intake.stopExtension();
    }
}