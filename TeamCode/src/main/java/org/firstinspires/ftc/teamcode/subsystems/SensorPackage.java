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
            // Pipelineswitch ?? makes sure it's in the correct pipeline... as opposed to..?
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
        }

    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }


    public void updatePose(Pose3D botpose){
        // Log pre-update pose
        telemetry.addData("Pre-Update Pose X", robot.mecanum.pose.position.x);
        telemetry.addData("Pre-Update Pose Y", robot.mecanum.pose.position.y);
        telemetry.addData("Pre-Update Heading", Math.toDegrees(robot.mecanum.pose.heading.toDouble()));

        // Log raw Limelight data
        telemetry.addData("Limelight X (m)", botpose.getPosition().x);
        telemetry.addData("Limelight Y (m)", botpose.getPosition().y);
        telemetry.addData("Limelight Heading (deg)", botpose.getOrientation().getYaw(AngleUnit.DEGREES));

        // TODO: should we stop the robot so we don't have unaccounted for momentum?
        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        double theta = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        if(theta < 0) theta += 360;
        // we're trying to update pose without nuking history
        robot.mecanum.pose = new Pose2d(x* 39.3701,y* 39.3701, Math.toRadians(theta));

        // Log post-update pose
        telemetry.addData("Post-Update Pose X", robot.mecanum.pose.position.x);
        telemetry.addData("Post-Update Pose Y", robot.mecanum.pose.position.y);
        telemetry.addData("Post-Update Heading", Math.toDegrees(robot.mecanum.pose.heading.toDouble()));
        robot.telemetry.addData("LL Theta",  theta);

        // Does this update actually erase our LL pose with problematic dead-wheel localization?
        robot.mecanum.updatePoseEstimate();

    }

    @Override
    public void periodic() {
        // APRILTAG LOCALIZATION
        if (limelight != null && aprilTagPositionTracking) {
            LLResult result = getLLResult();
            if(result != null && result.isValid()) {
                double tx = result.getTx(); // How far the tag is from center horizontally
                double ty = result.getTy(); // Vertical offset
                double ta = result.getTa(); // How big the tag looks, usually correlating to distance

                robot.telemetry.addData("Target X", tx);
                robot.telemetry.addData("Target Y", ty);
                robot.telemetry.addData("Target Area", ta);

                robot.telemetry.addData("target id", result.getFiducialResults());

                Pose3D botpose = result.getBotpose();
                // TODO: before we updatePose, check if there's enough of a difference
                if (botpose != null) {
                    robot.sensors.updatePose(botpose);
                }
            }
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