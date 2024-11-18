package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Callisto;
//import org.firstinspires.ftc.teamcode.commands.LiftRaise;


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