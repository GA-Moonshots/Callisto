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

    // Last detection metrics for debugging
    private long lastDetectionTime = 0;
    private double lastDetectionDistance = 0;
    private int totalDetections = 0;
    private int filteredDetections = 0;
    private int acceptedDetections = 0;

    public SensorPackage(Callisto robot) {
        super(robot);

        try {
            limelight = robot.hardwareMap.get(Limelight3A.class, Constants.LIMELIGHT_NAME);
            // Will lowering this slow battery drain
            limelight.setPollRateHz(100); // frequency of updates
            limelight.start();
            // Pipeline switch ? makes sure it's in the correct pipeline... as opposed to..?
            limelight.pipelineSwitch(0);
        } catch (Exception e) {
            telemetry.addData("Limelight Error", e.getMessage());
        }
    }

    public LLResult getLLResult() {
        return limelight.getLatestResult();
    }

    /**
     * Checks if a detected pose should be used to update the robot's position
     * based on distance and similarity criteria
     *
     * @param botpose The detected pose from Limelight
     * @return True if pose should be used, false if it should be ignored
     */
    private boolean shouldUpdatePose(Pose3D botpose) {
        // Convert position to inches for consistent units
        double xMeters = botpose.getPosition().x;
        double yMeters = botpose.getPosition().y;
        double zMeters = botpose.getPosition().z;

        // 1. Calculate distance to the detected AprilTag
        // Since the Limelight gives us field-centric coordinates, this is actually
        // measuring the length of the position vector, not the distance to the tag
        // We can use the Z position directly as an approximation for tag distance
        double tagDistance = Math.abs(zMeters * 39.3701); // Convert to inches
        lastDetectionDistance = tagDistance;

        // Apply distance threshold - ignore tags more than 36 inches away
        final double MAX_DISTANCE = 36.0; // inches
        if (tagDistance > MAX_DISTANCE) {
            telemetry.addData("AprilTag Filtered", "Too far (%.1f in)", tagDistance);
            filteredDetections++;
            return false;
        }

        // 2. Check if current pose and detected pose are already similar
        double xInches = xMeters * 39.3701;
        double yInches = yMeters * 39.3701;
        double heading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        if (heading < 0) heading += 360;

        // Get current RoadRunner pose
        Pose2d currentPose = robot.mecanum.pose;

        // Calculate differences
        double poseDiffX = Math.abs(xInches - currentPose.position.x);
        double poseDiffY = Math.abs(yInches - currentPose.position.y);
        double currentHeading = Math.toDegrees(currentPose.heading.toDouble());
        if (currentHeading < 0) currentHeading += 360;

        double poseDiffTheta = Math.abs(heading - currentHeading);
        if (poseDiffTheta > 180) poseDiffTheta = 360 - poseDiffTheta;

        // Define thresholds for position similarity
        final double POSITION_THRESHOLD = 1.0; // 1 inch
        final double HEADING_THRESHOLD = 5.0; // 5 degrees

        // Log the differences for debugging
        telemetry.addData("Pose Diff", "X: %.1f\" Y: %.1f\" θ: %.1f°",
                poseDiffX, poseDiffY, poseDiffTheta);

        // Check if poses are already similar
        if (poseDiffX < POSITION_THRESHOLD &&
                poseDiffY < POSITION_THRESHOLD &&
                poseDiffTheta < HEADING_THRESHOLD) {
            telemetry.addData("AprilTag Filtered", "Similar pose");
            filteredDetections++;
            return false;
        }

        // If we passed all checks, we should update the pose
        acceptedDetections++;
        return true;
    }

    public void updatePose(Pose3D botpose) {
       /* // Log pre-update pose
        telemetry.addData("Pre-Update Pose X", robot.mecanum.pose.position.x);
        telemetry.addData("Pre-Update Pose Y", robot.mecanum.pose.position.y);
        telemetry.addData("Pre-Update Heading", Math.toDegrees(robot.mecanum.pose.heading.toDouble()));

        // Log raw Limelight data
        telemetry.addData("Limelight X (m)", botpose.getPosition().x);
        telemetry.addData("Limelight Y (m)", botpose.getPosition().y);
        telemetry.addData("Limelight Heading (deg)", botpose.getOrientation().getYaw(AngleUnit.DEGREES)); */

        double x = botpose.getPosition().x;
        double y = botpose.getPosition().y;
        double theta = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
        if(theta < 0) theta += 360;

        robot.mecanum.pose = new Pose2d(x* 39.3701, y* 39.3701, Math.toRadians(theta));

        // Log post-update pose
        /*telemetry.addData("Post-Update Pose X", robot.mecanum.pose.position.x);
        telemetry.addData("Post-Update Pose Y", robot.mecanum.pose.position.y);
        telemetry.addData("Post-Update Heading", Math.toDegrees(robot.mecanum.pose.heading.toDouble()));
        robot.telemetry.addData("LL Theta", theta); */

        // Does this update actually erase our LL pose with problematic dead-wheel localization?
        robot.mecanum.updatePoseEstimate();
    }

    @Override
    public void periodic() {
        // APRILTAG LOCALIZATION
        if (limelight != null && aprilTagPositionTracking) {
            LLResult result = getLLResult();
            if(result != null && result.isValid()) {
                totalDetections++;
                lastDetectionTime = System.currentTimeMillis();

                double tx = result.getTx(); // How far the tag is from center horizontally
                double ty = result.getTy(); // Vertical offset
                double ta = result.getTa(); // How big the tag looks, usually correlating to distance

              /*  robot.telemetry.addData("Target X", tx);
                robot.telemetry.addData("Target Y", ty);
                robot.telemetry.addData("Target Area", ta); */

                robot.telemetry.addData("Target ID", result.getFiducialResults());

                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    // Only update pose if it passes our filtering logic
                    if (shouldUpdatePose(botpose)) {
                        updatePose(botpose);
                        telemetry.addData("AprilTag", "✓ Pose updated");
                    } else {
                        telemetry.addData("AprilTag", "✗ Update filtered");
                    }
                }
            }
        }

        // Display AprilTag detection statistics
        if (aprilTagPositionTracking) {
            long timeSinceLast = System.currentTimeMillis() - lastDetectionTime;
            telemetry.addData("AprilTag Stats",
                    "Total: %d, Filtered: %d, Applied: %d",
                    totalDetections, filteredDetections, acceptedDetections);

            if (lastDetectionTime > 0) {
                telemetry.addData("Last Detection",
                        timeSinceLast < 1000 ?
                                String.format("%dms ago (%.1f\")", timeSinceLast, lastDetectionDistance) :
                                "> 1s ago");
            }
        }

        // !!! THIS SHOULD BE THE ONLY TELEMETRY UPDATE IN THE WHOLE PROJECT !!! //
        telemetry.update();
    }

    public void enableAprilTagTracking() {
        aprilTagPositionTracking = true;
    }

    public void disableAprilTagTracking() {
        aprilTagPositionTracking = false;
    }

    /**
     * Resets the detection statistics
     */
    public void resetDetectionStats() {
        totalDetections = 0;
        filteredDetections = 0;
        acceptedDetections = 0;
    }
}