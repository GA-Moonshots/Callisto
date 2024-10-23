package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class ExtendArm extends CommandBase {
    private final Callisto callisto;
    private final Arm shoulderMotor;
    private final GamepadEx player2;

    public ExtendArm(Callisto callisto) {
        this.callisto = callisto;
        shoulderMotor = this.callisto.arm;

        player2 = callisto.player2;

        addRequirements(shoulderMotor);
    }

    @Override
    public void execute() {
        shoulderMotor.moveArm(player2.getLeftY());
    }
}
