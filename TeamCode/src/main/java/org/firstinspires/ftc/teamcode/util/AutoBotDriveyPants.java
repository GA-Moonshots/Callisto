package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Callisto;

/**
 * The primary operation file for the teleOp phase of the match
 */
@com.qualcomm.robotcore.eventloop.opmode.Autonomous(name = "Autonomous - Primary")
public class AutoBotDriveyPants extends CommandOpMode {
    public boolean isRed;
    public boolean left;

    /**
     * Set up robot such that it asks the player what our starting position is and kicks off
     * a FTCLib-style RoadRunner.
     */
    @Override
    public void initialize() {

        isRed = false;
        left = true;

        while(opModeInInit()) {
            // press X for blue and B for red
            if (gamepad1.x)
                isRed = false;
            else if (gamepad1.b && !gamepad1.start)
                isRed = true;
            // compound condition required bc of player selection
            if(gamepad1.a && !gamepad1.start){
                left = true;
            }
            else if (gamepad1.y){
                left = false;

            }

            // DISPLAY SELECTION
            telemetry.addData("Red Team:" , isRed );
            telemetry.addData("Left Side:", left);
            telemetry.update();
        }


        /*
         We build our robot. From here on out, we don't need this file. When we build the robot,
         all of our buttons are bound to commands and this class's parent, CommandOpMode, will
         continuously run any scheduled commands. We now slide into the WPILib style.
         We pass in our autonomous config variables, which signals to the robot we want to be in
         autonomous mode instead of in teleop mode, which would take no params besides this.
         */
        Robot m_robot = new Callisto(this, isRed, left);
    }

}


