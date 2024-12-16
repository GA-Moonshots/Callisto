package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;
import org.firstinspires.ftc.teamcode.util.experiments.AprilTagPositionTracker;
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

public class SensorPackage extends SubsystemBase {
    private Callisto robot;
    private final Telemetry telemetry;
    public Camera camera;

    private static final double MIN_CONFIDENCE_THRESHOLD = 0.5;
    private long lastUpdateTime = 0;
    private static final long MIN_UPDATE_INTERVAL_MS = 100;

    // Flag to enable/disable AprilTag position tracking
    public static boolean aprilTagPositionTracking = false;

    public SensorPackage(Callisto robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;

        try {
            this.camera = new Camera(robot, robot.opMode.telemetry);
        } catch (Exception ignored) {}
    }

    @Override
    public void periodic() {
        if (camera != null && aprilTagPositionTracking) {
            ArrayList<AprilTagDetection> detections = camera.getLatestFreshDetections();

            if (detections != null) {
                long currentTime = System.currentTimeMillis();

                if (currentTime - lastUpdateTime >= MIN_UPDATE_INTERVAL_MS) {
                    AprilTagDetection bestDetection = null;
                    double highestConfidence = MIN_CONFIDENCE_THRESHOLD;

                    for (AprilTagDetection detection : detections) {
                        double confidence = AprilTagPositionTracker.calculateConfidence(detection);
                        if (confidence > highestConfidence) {
                            highestConfidence = confidence;
                            bestDetection = detection;
                        }
                    }

                    if (bestDetection != null) {
                        robot.mecanum.pose = AprilTagPositionTracker.updatePoseEstimate(
                                bestDetection,
                                robot.mecanum.pose
                        );
                        lastUpdateTime = currentTime;

                        telemetry.addData("Position Updated", "Tag ID: " + bestDetection.id);
                        telemetry.addData("New Position", String.format("X: %.2f, Y: %.2f, Heading: %.2f",
                                robot.mecanum.pose.position.x,
                                robot.mecanum.pose.position.y,
                                Math.toDegrees(robot.mecanum.pose.heading.toDouble())));
                    }
                }
            }
        }

        telemetry.update();
    }

    /**
     * Enable AprilTag position tracking
     */
    public void enableAprilTagTracking() {
        aprilTagPositionTracking = true;
    }

    /**
     * Disable AprilTag position tracking
     */
    public void disableAprilTagTracking() {
        aprilTagPositionTracking = false;
    }
}