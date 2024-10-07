package org.firstinspires.ftc.teamcode.subsystems;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Blinkin extends SubsystemBase {
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern currentPattern;
    private Callisto robot;

    //constructor
    public Blinkin(Callisto callisto ) {
        blinkinLedDriver = callisto.hardwareMap.get(RevBlinkinLedDriver.class , Constants.LED_CONTROLLER);
        robot = callisto;
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern){

        currentPattern = pattern;
        blinkinLedDriver.setPattern(currentPattern);
    }


    //create a command that would change the colors

    public void changeColorIsTeamRed(boolean teamRed){
        robot.telemetry.addData("trying to change color", teamRed);
        if(teamRed){
            setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }else{
            setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);

        }
        robot.telemetry.update();
    }
}
