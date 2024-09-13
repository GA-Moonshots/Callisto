package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Callisto;


@TeleOp(name="TeleOp - Main")
public class DriveyMcDriverson extends CommandOpMode {

    @Override
    public void initialize() {
        boolean isWorking = true;
        telemetry.addData("Is it working: ", isWorking);
        telemetry.update();

        Robot m_robot = new Callisto(this);
        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         */
//        drive = new MecanumDrive(
//                new Motor(hardwareMap, "frontLeft"),
//                new Motor(hardwareMap, "frontRight"),
//                new Motor(hardwareMap, "backLeft"),
//                new Motor(hardwareMap, "backRight")
//        );
//
//        mecanumDriveSub = new MecanumDriveSub(drive);
//
//        register(mecanumDriveSub);
//        mecanumDriveSub.setDefaultCommand(new DriveCommand(mecanumDriveSub));
    }

}