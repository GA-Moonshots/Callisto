package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Limelight extends SubsystemBase {

    private Callisto robot;
    private Limelight3A limelight;

    public Limelight(Callisto robot) {
        this.robot = robot;
        limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.setPollRateHz(100);
        limelight.start();

        limelight.pipelineSwitch(0);
    }

    public void switchPipeline(int channel) {
        limelight.pipelineSwitch(channel);
    }
}
