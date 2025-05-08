package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;
import org.firstinspires.ftc.teamcode.util.experiments.AprilTagPositionTracker;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.ArrayList;

public class SensorPackage extends SubsystemBase {
    private final Callisto robot;
    private final Telemetry telemetry;
    public Camera camera;

    private static final double MIN_CONFIDENCE_THRESHOLD = 0.5;
    private long lastUpdateTime = 0;
    private static final long MIN_UPDATE_INTERVAL_MS = 100; // Update position at most every 100ms

    // Flag to enable/disable AprilTag position tracking
    public boolean aprilTagPositionTracking = false;

    public SensorPackage(Callisto robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;

        try {
            this.camera = new Camera(robot, robot.opMode.telemetry);
        } catch (Exception e) {
            telemetry.addData("Camera Error", e.getMessage());
            telemetry.update();
        }
    }

    @Override
    public void periodic() {
        if (camera != null && aprilTagPositionTracking) {
            ArrayList<AprilTagDetection> detections = camera.getLatestFreshDetections();

            if (detections != null && !detections.isEmpty()) {
                long currentTime = System.currentTimeMillis();

                if (currentTime - lastUpdateTime >= MIN_UPDATE_INTERVAL_MS) {
                    // Find the most confident detection
                    AprilTagDetection bestDetection = findBestDetection(detections);

                    if (bestDetection != null) {
                        // Update robot pose based on the AprilTag detection
                        robot.mecanum.pose = AprilTagPositionTracker.updatePoseEstimate(
                                bestDetection,
                                robot.mecanum.pose
                        );
                        lastUpdateTime = currentTime;

                        // Log the update
                        telemetry.addData("Position Updated", "Tag ID: " + bestDetection.id);
                        telemetry.addData("Tag Range", String.format("%.2f inches", bestDetection.ftcPose.range));
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
     * Find the most reliable AprilTag detection from a list of detections
     * @param detections List of AprilTag detections
     * @return The most reliable detection, or null if none meet the confidence threshold
     */
    private AprilTagDetection findBestDetection(ArrayList<AprilTagDetection> detections) {
        AprilTagDetection bestDetection = null;
        double highestConfidence = MIN_CONFIDENCE_THRESHOLD;

        for (AprilTagDetection detection : detections) {
            double confidence = calculateConfidence(detection);
            if (confidence > highestConfidence) {
                highestConfidence = confidence;
                bestDetection = detection;
            }
        }

        return bestDetection;
    }

    /**
     * Calculate a confidence score for an AprilTag detection
     * @param detection The AprilTag detection to evaluate
     * @return Confidence score between 0 and 1
     */
    private double calculateConfidence(AprilTagDetection detection) {
        // Use decision margin as primary confidence metric
        double decisionMarginConfidence = Math.min(1.0, detection.decisionMargin / 20.0);

        // Factor in distance (closer tags are generally more reliable)
        double range = detection.ftcPose.range;
        double rangeConfidence = Math.max(0, 1.0 - (range / 100.0)); // Decreased confidence beyond 100 inches

        // Combine metrics (weighing decision margin more heavily)
        return (decisionMarginConfidence * 0.7) + (rangeConfidence * 0.3);
    }

    public void enableAprilTagTracking() {
        aprilTagPositionTracking = true;
    }

    public void disableAprilTagTracking() {
        aprilTagPositionTracking = false;
    }
}