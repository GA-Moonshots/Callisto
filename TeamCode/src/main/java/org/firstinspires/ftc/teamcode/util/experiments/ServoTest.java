package org.firstinspires.ftc.teamcode.util.experiments;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class ServoTest extends SubsystemBase {
    private Servo servo;     //create a class member of type Servo named servo. The Servo class comes from the FTC SDK
    public boolean isOpen = false;
    //static

    // constructor  for the servo
    public ServoTest(Callisto callisto){
        servo = callisto.hardwareMap.get(Servo.class, Constants.WRIST_SERVO_NAME);
    }

    public void open() {
        servo.setPosition(1.0);  // Range is from 0.0 to 1.0
    }
    public void close() {
        servo.setPosition(0.0);  // Range is from 0.0 to 1.0
    }

}








