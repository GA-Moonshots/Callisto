package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.openftc.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.Map;

public class AprilTagPositionTracker {
    // Camera mounting constants - adjust these based on tuning results
    public static double CAMERA_X_OFFSET = 0.0;  // Left/Right from robot center (inches)
    public static double CAMERA_Y_OFFSET = 0.0;  // Forward/Back from robot center (inches)
    public static double CAMERA_Z_OFFSET = 0.0;  // Up/Down from robot center (inches)
    public static double CAMERA_YAW_OFFSET = 0.0; // Rotation offset in degrees

    // Constants for field dimensions and tag heights
    private static final double TAG_HEIGHT = 5.9; // Height of tags above the field in inches

    // Store AprilTag positions on the field (ID -> Position)
    private static final Map<Integer, Pose2d> TAG_POSITIONS = new HashMap<Integer, Pose2d>() {{
        // Red Alliance Side
        put(15, new Pose2d(new Vector2d(0, 72), Math.toRadians(180)));     // Red Side Wall
        put(16, new Pose2d(new Vector2d(-72, 36), Math.toRadians(90)));    // Red Scoring End
        put(14, new Pose2d(new Vector2d(-72, -36), Math.toRadians(90)));   // Red Loading End

        // Blue Alliance Side
        put(12, new Pose2d(new Vector2d(0, -72), Math.toRadians(0)));      // Blue Side Wall
        put(13, new Pose2d(new Vector2d(72, 36), Math.toRadians(-90)));    // Blue Scoring End
        put(11, new Pose2d(new Vector2d(72, -36), Math.toRadians(-90)));   // Blue Loading End
    }};

    /**
     * Extract yaw angle from rotation matrix
     * Assumes rotation matrix is in standard form where:
     * R = [r11 r12 r13]
     *     [r21 r22 r23]
     *     [r31 r32 r33]
     * yaw = atan2(r21, r11)
     */
    private static double getYawFromRotationMatrix(MatrixF R) {
        return Math.atan2(R.get(1, 0), R.get(0, 0));
    }

    /**
     * Calculates robot position based on AprilTag detection
     */
    public static Pose2d calculateRobotPosition(AprilTagDetection detection) {
        if (!TAG_POSITIONS.containsKey(detection.id)) {
            return null;
        }

        Pose2d tagPose = TAG_POSITIONS.get(detection.id);

        // Get the translation vector from the camera to the tag
        double x = detection.pose.x;
        double y = detection.pose.y;
        double z = detection.pose.z;

        // Get the yaw angle from the rotation matrix
        double cameraYaw = getYawFromRotationMatrix(detection.pose.R);

        // Calculate total distance to tag in XZ plane
        double distance = Math.sqrt(x * x + z * z);

        // Calculate angle to tag in camera frame
        double angleToTag = Math.atan2(x, z);

        // Apply camera mounting offsets
        double offsetAngle = Math.toRadians(CAMERA_YAW_OFFSET);
        double robotYaw = tagPose.heading.toDouble() - cameraYaw + offsetAngle;

        // Calculate robot's position relative to the tag
        double robotX = tagPose.position.x - (distance * Math.cos(tagPose.heading.toDouble() + angleToTag));
        double robotY = tagPose.position.y - (distance * Math.sin(tagPose.heading.toDouble() + angleToTag));

        // Apply camera position offsets
        robotX += CAMERA_X_OFFSET * Math.cos(robotYaw) - CAMERA_Y_OFFSET * Math.sin(robotYaw);
        robotY += CAMERA_X_OFFSET * Math.sin(robotYaw) + CAMERA_Y_OFFSET * Math.cos(robotYaw);

        return new Pose2d(new Vector2d(robotX, robotY), robotYaw);
    }

    /**
     * Updates RoadRunner's position estimate based on AprilTag detection
     */
    public static Pose2d updatePoseEstimate(AprilTagDetection detection, Pose2d currentPose) {
        Pose2d tagBasedPose = calculateRobotPosition(detection);
        if (tagBasedPose == null) {
            return currentPose;
        }

        double confidence = calculateConfidence(detection);

        double newX = (currentPose.position.x * (1 - confidence)) + (tagBasedPose.position.x * confidence);
        double newY = (currentPose.position.y * (1 - confidence)) + (tagBasedPose.position.y * confidence);
        double newHeading = (currentPose.heading.toDouble() * (1 - confidence)) + (tagBasedPose.heading.toDouble() * confidence);

        return new Pose2d(new Vector2d(newX, newY), newHeading);
    }

    /**
     * Calculates confidence in the AprilTag detection based on distance and visibility
     */
    public static double calculateConfidence(AprilTagDetection detection) {
        // Calculate distance to tag
        double distance = Math.sqrt(
                detection.pose.x * detection.pose.x +
                        detection.pose.y * detection.pose.y +
                        detection.pose.z * detection.pose.z
        );

        // Confidence decreases with distance (max reliable distance around 60 inches)
        double distanceConfidence = Math.max(0, Math.min(1, 1 - (distance / 60)));

        // Calculate angle confidence using rotation matrix determinant
        // A determinant closer to 1 indicates a better view of the tag
        double determinant = Math.abs(detection.pose.R.get(0, 0) *
                detection.pose.R.get(1, 1) *
                detection.pose.R.get(2, 2));
        double angleConfidence = Math.max(0, Math.min(1, determinant));

        return Math.max(0, Math.min(1, (distanceConfidence + angleConfidence) / 2));
    }
}