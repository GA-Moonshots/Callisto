package org.firstinspires.ftc.teamcode.commands;

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
    private final double targetYaw;
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
        this.targetY = targetY;
        this.targetYaw = 0; // Adjust as necessary for your use case
        this.tagId = tagId;
        this.dashboard = FtcDashboard.getInstance();
        this.timer = new Timing.Timer(DEFAULT_TIMEOUT);
        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        timer.start();
        finished = false;
    }

    @Override
    public void execute() {
        AprilTagDetection detection = robot.sensors.camera.getLatestFreshDetections().get(0);
        if (detection != null) {
            if (detection.id == tagId) {
                    double currentX = detection.ftcPose.x;
                    double currentY = detection.ftcPose.y;

                    double xError = targetX - currentX;
                    double yError = targetY - currentY;

                    double xSpeed = Math.max(-MAX_ADJUSTMENT_SPEED, Math.min(MAX_ADJUSTMENT_SPEED, xError));
                    double ySpeed = Math.max(-MAX_ADJUSTMENT_SPEED, Math.min(MAX_ADJUSTMENT_SPEED, yError));

                    mecanum.drive(xSpeed, ySpeed, 0);

                    if (Math.abs(xError) < POSITION_TOLERANCE && Math.abs(yError) < POSITION_TOLERANCE) {
                        finished = true;
                        mecanum.stop();
                        return;
                    }

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.put("X Error", xError);
                    packet.put("Y Error", yError);

                    dashboard.sendTelemetryPacket(packet);

                    return;

            }
        }
    }

    @Override
    public boolean isFinished() {
        return finished;
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.stop();
    }
}
