
package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

public class IntakeShoulderDown extends CommandBase {
    private final Callisto robot;
    private final GamepadEx player2;

    private final Intake intake;

    private int targetPosition = 5;
    private double power = 0.15;

    private final double TIMEOUT = 3.0; // seconds
    protected Timing.Timer timer;

    public IntakeShoulderDown(Callisto robot) {
        this.robot = robot;
        this.intake = robot.intake;

        player2 = robot.player2;

        timer = new Timing.Timer((long) TIMEOUT);

        // addRequirements(this.robot.intake);
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("wants to be at position :",targetPosition);
        intake.shoulderMotor.set(-0.2);
    }
    @Override
    public void end(boolean interrupted) {
        intake.shoulderMotor.stopMotor();
        intake.isShoulderUp = !intake.isShoulderUp;
    }
}