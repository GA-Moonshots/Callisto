package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.CommandOpMode;
import com.seattlesolvers.solverslib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Callisto;


@TeleOp(name="TeleOp - Main")
public class DriveyMcDriverson extends CommandOpMode {

    @Override
    public void initialize() {
        boolean isWorking = true;
        telemetry.addData("Is it working: ", isWorking);
        telemetry.update();

        Callisto m_robot = new Callisto(this);
    }
}