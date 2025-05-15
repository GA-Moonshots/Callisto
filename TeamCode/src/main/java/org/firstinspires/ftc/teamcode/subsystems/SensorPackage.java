package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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


    public void updatePose(Pose3D botpose){
        // TODO: should we stop the robot so we don't have unaccounted for momentum?
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        // replace with new wrapped theta

        double theta = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        if(theta < 0) theta += 360;

        //telemetry.addData("sensors:", "nuking history");
        robot.mecanum.nukeHistory();
        robot.mecanum.pose = new Pose2d(x* 39.3701,y* 39.3701, Math.toRadians(theta));

        robot.mecanum.updatePoseEstimate();


        robot.telemetry.addData("LL Theta",  theta);

    }

    @Override
    public void periodic() {
        if (limelight != null && aprilTagPositionTracking) {
            // TODO: Move command's execute logic here
            // TODO: before we updatePose, check if there's enough of a difference
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