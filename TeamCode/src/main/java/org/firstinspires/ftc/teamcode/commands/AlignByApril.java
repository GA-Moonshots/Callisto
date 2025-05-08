package org.firstinspires.ftc.teamcode.commands;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;

public class AlignByApril extends CommandBase {
    private final Callisto robot;
    private final Mecanum mecanum;
    private final double targetX;
    private final double targetY;
    private final double mecX;
    private final double mecY;
    private boolean finished = false;
    private final FtcDashboard dashboard;
    private final Timing.Timer timer;

    private static final double POSITION_TOLERANCE = 0.75; // inches
    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double MAX_ADJUSTMENT_SPEED = 0.3;
    private static final long DEFAULT_TIMEOUT = 3; // 5 seconds timeout

    private double currentX;
    private double currentY;


    private double yDelta;
    private double xDelta;

    private AprilTagDetection detection;

    public AlignByApril(Callisto robot, double targetX, double targetY, double mecX, double mecY) {
        this.robot = robot;
        this.mecanum = robot.mecanum;

        this.targetX = targetX;
        this.targetY = targetY;

        this.mecX = mecX;
        this.mecY = mecY;

        // Adjust as necessary for your use case
        this.dashboard = FtcDashboard.getInstance();
        this.timer = new Timing.Timer(DEFAULT_TIMEOUT);
        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        timer.start();
        //mecanum.setGyroLocked();
        robot.telemetry.addData("Align Started", true);
    }

    @Override
    public void execute() {

        try {
            detection = robot.sensors.camera.getLatestFreshDetections().get(0);
        } catch (Exception e) {
            robot.telemetry.addLine("Detection failed");
            detection = null;
        }

        if (detection != null) {
            // Get the pose from the April Tag detection

            currentX = detection.ftcPose.x;
            currentY = detection.ftcPose.y;

            // AprilTag y and x are opposite of RRs
            yDelta = Math.abs(targetX - currentX);
            xDelta = Math.abs(targetY - currentY);

            new StrafeToPose(robot, new Pose2d(new Vector2d(mecX - xDelta,
                    mecY + yDelta),
                    mecanum.pose.heading)
            ).schedule();
        }

        robot.telemetry.addData("Tag X: ", currentX);
        robot.telemetry.addData("Tag Y: ", currentY);

        robot.telemetry.addData("Target X: ", targetX);
        robot.telemetry.addData("Target Y: ", targetY);

        robot.telemetry.addData("Y Delta", yDelta);
        robot.telemetry.addData("X Delta", xDelta);

        robot.telemetry.addData("X position: ", mecanum.pose.position.x);
        robot.telemetry.addData("Y position: ", mecanum.pose.position.y);
    }

    @Override
    public boolean isFinished() {
        if(detection != null) {
            return ((Math.abs(detection.ftcPose.x - targetX) < POSITION_TOLERANCE) &&
                    (Math.abs(detection.ftcPose.y - targetY) < POSITION_TOLERANCE)) ||
                    timer.done();
        }
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.setGyroUnlocked();
        mecanum.stop();
    }
}