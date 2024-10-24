package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Arm;

public class RaiseShoulder extends CommandBase {
    private final Callisto callisto;
    private final Arm shoulderMotor;
    private final GamepadEx player2;

    public RaiseShoulder(Callisto callisto) {
        this.callisto = callisto;
        shoulderMotor = this.callisto.arm;

        player2 = callisto.player2;

        addRequirements(shoulderMotor);
    }

    @Override
    public void execute() {
        // TODO: include dead zone for joystick input
        shoulderMotor.moveShoulder(player2.getRightY());
    }

    // TODO: include an end function to safely power down any motion
}
