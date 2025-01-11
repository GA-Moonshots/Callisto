package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.HashMap;
import java.util.Map;

public class AprilTagPositionTracker {
    // Camera mounting constants - adjust these based on tuning results
    public static double CAMERA_X_OFFSET = 0.0;  // Left/Right from robot center (inches)
    public static double CAMERA_Y_OFFSET = 0.0;  // Forward/Back from robot center (inches)
    public static double CAMERA_HEIGHT = 11.0;   // Height of camera from ground (inches)
    public static double CAMERA_YAW_OFFSET = 0.0; // Rotation offset in degrees

    // Field constants
    public static double APRILTAG_HEIGHT = 5.9;  // Height of AprilTag centers from ground (inches)

    // Derived constant - vertical distance between camera and tag centers
    public static double HEIGHT_DIFFERENTIAL = CAMERA_HEIGHT - APRILTAG_HEIGHT;

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
     * Calculates robot position based on AprilTag detection.
     * Using fixed camera height and tag height, we can calculate position more accurately.
     *
     * For a camera mounted parallel to the ground:
     * - Pitch = 0° when looking straight ahead
     * - Positive pitch when looking up
     * - Negative pitch when looking down
     *
     * The actual ground distance to the tag is calculated using:
     * - The measured direct distance (range)
     * - The known height difference between camera and tag
     * - The pitch angle of the detection
     */
    public static Pose2d calculateRobotPosition(AprilTagDetection detection) {
        if (!TAG_POSITIONS.containsKey(detection.id)) {
            return null;
        }

        Pose2d tagPose = TAG_POSITIONS.get(detection.id);

        double distance = detection.ftcPose.range;
        double pitch = Math.toRadians(detection.ftcPose.pitch);
        double yaw = Math.toRadians(detection.ftcPose.yaw);

        // Calculate robot's position relative to the tag
        // Since the camera is fixed straight ahead, pitch helps us determine:
        // - relativeY: the forward distance (adjacent to pitch angle)
        // - relativeX: the side-to-side distance based on yaw
        double relativeY = distance * Math.cos(pitch);
        double relativeX = distance * Math.sin(yaw);

        // We can verify our distance calculation using the height differential
        double calculatedDistance = HEIGHT_DIFFERENTIAL / Math.sin(pitch);
        // If the calculated and measured distances differ significantly,
        // we might want to adjust our confidence in this measurement

        // Calculate robot's absolute heading
        double robotYaw = tagPose.heading.toDouble() - yaw + Math.toRadians(CAMERA_YAW_OFFSET);

        // Calculate robot position in field coordinates
        double robotX = tagPose.position.x - (relativeX * Math.cos(tagPose.heading.toDouble()) -
                relativeY * Math.sin(tagPose.heading.toDouble()));
        double robotY = tagPose.position.y - (relativeX * Math.sin(tagPose.heading.toDouble()) +
                relativeY * Math.cos(tagPose.heading.toDouble()));

        // Apply camera position offsets to account for camera mounting position
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

        // Calculate confidence based on detection quality
        double confidence = calculateConfidence(detection);

        // Weighted average between current pose and tag-based pose
        double newX = (currentPose.position.x * (1 - confidence)) + (tagBasedPose.position.x * confidence);
        double newY = (currentPose.position.y * (1 - confidence)) + (tagBasedPose.position.y * confidence);
        double newHeading = (currentPose.heading.toDouble() * (1 - confidence)) + (tagBasedPose.heading.toDouble() * confidence);

        return new Pose2d(new Vector2d(newX, newY), newHeading);
    }

    /**
     * Calculates confidence in the AprilTag detection based on:
     * - Distance (closer is better)
     * - Decision margin (higher is better)
     * - Height differential accuracy (comparing measured vs calculated distances)
     */
    public static double calculateConfidence(AprilTagDetection detection) {
        // Distance confidence (max reliable distance around 60 inches)
        double distanceConfidence = Math.max(0, Math.min(1, 1 - (detection.ftcPose.range / 60)));

        // Decision margin confidence (normalized to 0-1 range)
        double marginConfidence = Math.min(1.0, detection.decisionMargin / 20.0);

        // Calculate confidence based on height differential accuracy
        double pitch = Math.toRadians(detection.ftcPose.pitch);
        double calculatedDistance = HEIGHT_DIFFERENTIAL / Math.sin(pitch);
        double measuredDistance = detection.ftcPose.range;
        double distanceError = Math.abs(calculatedDistance - measuredDistance) / measuredDistance;
        double heightConfidence = Math.max(0, 1 - distanceError);

        // Weight the confidence factors
        return Math.max(0, Math.min(1,
                (distanceConfidence * 0.3) +
                        (marginConfidence * 0.4) +
                        (heightConfidence * 0.3)));
    }
}