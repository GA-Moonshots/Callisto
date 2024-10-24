package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ExtendShoulderByAmount extends CommandBase {
    private Callisto robot;
    private Arm arm;

    private double amount;

    public ExtendShoulderByAmount(Callisto callisto, double amount) {
        robot = callisto;
        arm = callisto.arm;

        this.amount = amount;

        addRequirements(arm);
    }

    @Override
    public void execute() {
        arm.moveShoulder(amount);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveShoulder(0);
    }
}
