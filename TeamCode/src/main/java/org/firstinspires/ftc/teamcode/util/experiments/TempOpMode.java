package org.firstinspires.ftc.teamcode.util.experiments;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Callisto;

import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;

@TeleOp(name = "Temp AprilTag Camera Tuning", group = "tuning")
public class TempOpMode extends LinearOpMode {
    private Callisto robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Callisto(this);
        AprilTagDetection april = robot.sensors.camera.getLatestFreshDetections().get(0);

        telemetry.addData("X ", april.ftcPose.x);
        telemetry.addData("Y ", april.ftcPose.y);
        telemetry.addData("Z ", april.ftcPose.z);

        telemetry.addData("Bearing angle", april.ftcPose.bearing);
        telemetry.addData("Roll", april.ftcPose.roll);
        telemetry.addData("Yaw", april.ftcPose.yaw);
        telemetry.addData("Elevation", april.ftcPose.elevation);
        telemetry.addData("Pitch", april.ftcPose.pitch);

        double distance = Math.sqrt(Math.pow(april.ftcPose.x, 2) + Math.pow(april.ftcPose.y, 2) + Math.pow(april.ftcPose.z, 2));

        telemetry.addData("Distance", distance);


    }
}
