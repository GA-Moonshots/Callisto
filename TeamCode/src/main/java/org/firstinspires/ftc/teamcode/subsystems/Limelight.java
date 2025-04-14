package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Limelight extends SubsystemBase {

    private Callisto robot;
    private Limelight3A limelight;
    private Telemetry telemetry;

    private double x, y, theta;

    public Limelight(Callisto robot) {
        this.robot = robot;
        limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
        limelight.setPollRateHz(100);

        this.telemetry = robot.telemetry;
        this.telemetry.addData("Limelight initialize attempted", true);
        this.telemetry.update();

        limelight.start();

        limelight.pipelineSwitch(0);
    }

    public void switchPipeline(int channel) {
        limelight.pipelineSwitch(channel);
    }

    public LLResult limelightRun() {
        LLResult result = limelight.getLatestResult();
        return result;
    }

    public void passInValues(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public void periodic() {
        robot.mecanum.pose = new Pose2d(x,y,theta);
        robot.mecanum.updatePoseEstimate();
    }
}
