 package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Lift extends SubsystemBase {

    private Callisto robot;
// set up all the different motors
    public Lift(Callisto callisto){
        // lift = callisto.hardwareMap.get(RevBlinkinLedDriver.class , Constants.LED_CONTROLLER);
        robot = callisto;

    }

    public void up(){}

    public void down(){}

    public void yeet(){}
}

