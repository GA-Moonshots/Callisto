package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MoonBase;

public class SensorPackage extends MoonBase {
    private Limelight3A limelight;
    private double x, y, theta;

    // Flag to enable/disable AprilTag position tracking
    public boolean aprilTagPositionTracking = false;

    public SensorPackage(Callisto robot) {
        super(robot);

        try {
            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
            limelight.setPollRateHz(100);
            limelight.start();
            // why pipelineswitch ??
            // just to make sure its in the correct pipeline
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
        }

    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    public void passInValues(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    @Override
    public void periodic() {
        if (limelight != null && aprilTagPositionTracking) {
            robot.mecanum.pose = new Pose2d(x,y,theta);
        }
        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!!
        telemetry.update();
    }


    public void enableAprilTagTracking() {
        aprilTagPositionTracking = true;
    }

    public void disableAprilTagTracking() {
        aprilTagPositionTracking = false;
    }
}