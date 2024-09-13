package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Callisto;

/**
 * The primary operation file for the teleOp phase of the match
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous - Primary")
public class AutoBotDriveyPants extends CommandOpMode {


    /**
     * Set up robot such that it asks the player what our starting position is and kicks off
     * a FTCLib-style RoadRunner.
     */
    @Override
    public void initialize() {

        boolean isRed = false;

        while(opModeInInit()) {
            // press X for blue and B for red
            if (gamepad1.x)
                isRed = false;
            else if (gamepad1.b && !gamepad1.start)
                isRed = true;

            // DISPLAY SELECTION
            telemetry.addData("Position", "%s Team, %s Side", isRed ? "Red" : "Blue" );
            telemetry.update();
        }


        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         We pass in our autonomous config variables, which signals to the robot we want to be in
         autonomous mode instead of in teleop mode, which would take no params besides this.
         */
        Robot m_robot = new Callisto(this);
    }

}


