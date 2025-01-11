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
    public static double CAMERA_HEIGHT = 9.0;   // Height of camera from ground (inches)
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
     *
     * For a camera mounted parallel to the ground:
     * - Pitch tells us the vertical angle to the tag
     *   - 0° when looking straight ahead at same height
     *   - Positive when looking up
     *   - Negative when looking down
     * - Yaw tells us the horizontal angle to the tag
     *   - 0° when the tag is directly ahead
     *   - Positive when tag is to the right
     *   - Negative when tag is to the left
     *
     * The position calculation uses:
     * 1. Pitch angle to determine how much of the measured distance is horizontal vs vertical
     * 2. Yaw angle to split the horizontal distance into X and Y components
     * 3. Known height differences to validate our measurements
     */
    public static Pose2d calculateRobotPosition(AprilTagDetection detection) {
        if (!TAG_POSITIONS.containsKey(detection.id)) {
            return null;
        }

        Pose2d tagPose = TAG_POSITIONS.get(detection.id);

        double distance = detection.ftcPose.range;
        double pitch = Math.toRadians(detection.ftcPose.pitch);
        double yaw = Math.toRadians(detection.ftcPose.yaw);

        // First, use pitch to find the horizontal distance to the tag
        // This is the distance if we were to project everything onto the ground plane
        double horizontalDistance = distance * Math.cos(pitch);

        // Now split this horizontal distance into X and Y components using pitch
        // Since our camera is fixed straight ahead:
        // - The X component (left/right) comes from the pitch angle
        // - The Y component (forward/back) is what remains
        double relativeX = horizontalDistance * Math.sin(pitch);
        double relativeY = horizontalDistance * Math.cos(pitch);

        // We can verify our distance calculation using the height differential
        // The vertical component should match our known height difference
        double verticalComponent = distance * Math.sin(pitch);
        double expectedVertical = HEIGHT_DIFFERENTIAL;

        // The closer these values match, the more confident we can be in our measurement
        // We use this in our confidence calculation

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
     * - Height differential accuracy (comparing measured vs calculated heights)
     */
    public static double calculateConfidence(AprilTagDetection detection) {
        // Distance confidence (max reliable distance around 60 inches)
        double distanceConfidence = Math.max(0, Math.min(1, 1 - (detection.ftcPose.range / 60)));

        // Decision margin confidence (normalized to 0-1 range)
        double marginConfidence = Math.min(1.0, detection.decisionMargin / 20.0);

        // Calculate height-based confidence
        double pitch = Math.toRadians(detection.ftcPose.pitch);
        double measuredVertical = detection.ftcPose.range * Math.sin(pitch);
        double heightError = Math.abs(measuredVertical - HEIGHT_DIFFERENTIAL) / HEIGHT_DIFFERENTIAL;
        double heightConfidence = Math.max(0, 1 - heightError);

        // Weight the confidence factors
        return Math.max(0, Math.min(1,
                (distanceConfidence * 0.3) +
                        (marginConfidence * 0.4) +
                        (heightConfidence * 0.3)));
    }
}