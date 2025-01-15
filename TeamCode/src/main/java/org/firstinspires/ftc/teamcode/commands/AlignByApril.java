package org.firstinspires.ftc.teamcode.commands;

import android.sax.StartElementListener;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

public class AlignByApril extends CommandBase {
    private final Callisto robot;
    private final Mecanum mecanum;
    private final double targetX;
    private final double targetY;
    private final int tagId;
    private boolean finished = false;
    private final FtcDashboard dashboard;
    private final Timing.Timer timer;

    private static final double POSITION_TOLERANCE = 0.5; // inches
    private static final double ANGLE_TOLERANCE = 2.0; // degrees
    private static final double MAX_ADJUSTMENT_SPEED = 0.3;
    private static final long DEFAULT_TIMEOUT = 5000;
    // 5 seconds timeout

    public AlignByApril(Callisto robot, int tagId, double targetX, double targetY) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.targetX = targetX;
        this.targetY = targetY;// Adjust as necessary for your use case
        this.tagId = tagId;
        this.dashboard = FtcDashboard.getInstance();
        this.timer = new Timing.Timer(DEFAULT_TIMEOUT);
        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        timer.start();
        mecanum.setGyroLocked();
        robot.telemetry.addData("Align Started", true);
        finished = false;
    }

    @Override
    public void execute() {
        AprilTagDetection detection = null;

        try {
            detection = robot.sensors.camera.getLatestFreshDetections().get(0);
        } catch (Exception e) {
            robot.telemetry.addLine("detection failed");
        }

        if (detection != null) {
            if (detection.id == tagId) {
                    double currentX = detection.ftcPose.x;
                    double currentY = detection.ftcPose.y;

                    robot.telemetry.addData("X: ", currentX);
                    robot.telemetry.addData("Y: ", currentY);

                    double xError = targetX - currentX;
                    double yError = targetY - currentY;

                    double xSpeed = .1;
                    double ySpeed = -.2;

                    mecanum.drive(xSpeed, ySpeed, 0);

                    if (Math.abs(xError) < POSITION_TOLERANCE){

                        mecanum.drive(0, ySpeed, 0);

                        if (Math.abs(yError) < POSITION_TOLERANCE){
                            mecanum.stop();
                            finished = true;

                        }
                    }


                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("X Error", xError);
                    packet.put("Y Error", yError);

                    dashboard.sendTelemetryPacket(packet);
            }
        }
    }

    @Override
    public boolean isFinished() {

        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.setGyroUnlocked();
        mecanum.stop();
    }
}
