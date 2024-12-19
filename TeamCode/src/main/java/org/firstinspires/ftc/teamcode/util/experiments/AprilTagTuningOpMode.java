package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Callisto;
//<<<<<<< HEAD
//import org.firstinspires.ftc.teamcode.util.AprilTagPositionTracker;
//=======
import org.firstinspires.ftc.teamcode.util.experiments.AprilTagPositionTracker;
//>>>>>>> fe8c5807a27b19e09370750ec94235cfffb56234
import org.openftc.apriltag.AprilTagDetection;
import java.util.ArrayList;

@TeleOp(name = "AprilTag Camera Tuning", group = "tuning")
public class AprilTagTuningOpMode extends LinearOpMode {
    // Define known positions to test with - adjust these based on your field setup
    private static final class CalibrationPoint {
        public final Pose2d pose;
        public final String description;

        public CalibrationPoint(Pose2d pose, String description) {
            this.pose = pose;
            this.description = description;
        }
    }

    private static final CalibrationPoint[] CALIBRATION_POINTS = {
            // Center of field positions
            new CalibrationPoint(
                    new Pose2d(new Vector2d(0, 0), 0),
                    "Field Center, Facing East (Right)"
            ),
            new CalibrationPoint(
                    new Pose2d(new Vector2d(0, 0), Math.toRadians(90)),
                    "Field Center, Facing North (Back Wall)"
            ),

            // Red Alliance positions
            new CalibrationPoint(
                    new Pose2d(new Vector2d(-24, 24), Math.toRadians(90)),
                    "Red Alliance Quarter, Facing North"
            ),

            // Blue Alliance positions
            new CalibrationPoint(
                    new Pose2d(new Vector2d(24, -24), Math.toRadians(-90)),
                    "Blue Alliance Quarter, Facing South"
            )
    };

    private int currentPointIndex = 0;
    private Callisto robot;

    @Override
    public void runOpMode() {
        robot = new Callisto(this);

        telemetry.addLine("AprilTag Camera Tuning OpMode");
        telemetry.addLine("Place robot at the first calibration position:");
        telemetry.addData("Position", CALIBRATION_POINTS[0].description);
        telemetry.addLine("Current camera offset values:");
        telemetry.addData("CAMERA_X_OFFSET", AprilTagPositionTracker.CAMERA_X_OFFSET);
        telemetry.addData("CAMERA_Y_OFFSET", AprilTagPositionTracker.CAMERA_Y_OFFSET);
        telemetry.addData("CAMERA_Z_OFFSET", AprilTagPositionTracker.CAMERA_Z_OFFSET);
        telemetry.addData("CAMERA_YAW_OFFSET", AprilTagPositionTracker.CAMERA_YAW_OFFSET);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            ArrayList<AprilTagDetection> detections = robot.sensors.camera.getLatestFreshDetections();

            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    // Get the actual pose calculated by our current implementation
                    Pose2d calculatedPose = AprilTagPositionTracker.calculateRobotPosition(detection);

                    if (calculatedPose != null) {
                        // Get the expected pose for current calibration point
                        CalibrationPoint currentPoint = CALIBRATION_POINTS[currentPointIndex];

                        // Calculate differences
                        double xDiff = calculatedPose.position.x - currentPoint.pose.position.x;
                        double yDiff = calculatedPose.position.y - currentPoint.pose.position.y;
                        double headingDiff = Math.toDegrees(
                                calculatedPose.heading.toDouble() - currentPoint.pose.heading.toDouble());

                        // Normalize heading difference to -180 to 180
                        while (headingDiff > 180) headingDiff -= 360;
                        while (headingDiff < -180) headingDiff += 360;

                        // Display results
                        telemetry.addLine("\nCalibration Results:");
                        telemetry.addData("Current Position", currentPoint.description);
                        telemetry.addData("Tag ID", detection.id);
                        telemetry.addData("Raw Tag Distance", String.format("%.1f inches",
                                Math.sqrt(detection.pose.x * detection.pose.x +
                                        detection.pose.y * detection.pose.y +
                                        detection.pose.z * detection.pose.z)));

                        telemetry.addLine("\nPosition Comparison:");
                        telemetry.addData("Expected", formatPose(currentPoint.pose));
                        telemetry.addData("Calculated", formatPose(calculatedPose));

                        telemetry.addLine("\nSuggested Adjustments:");
                        telemetry.addData("X Offset Adjustment", String.format("%.1f inches", -xDiff));
                        telemetry.addData("Y Offset Adjustment", String.format("%.1f inches", -yDiff));
                        telemetry.addData("Yaw Offset Adjustment", String.format("%.1f degrees", -headingDiff));
                    }
                }
            } else {
                telemetry.addLine("No AprilTags detected");
            }

            telemetry.addLine("\nControls:");
            telemetry.addLine("A - Next calibration point");
            telemetry.addLine("B - Previous calibration point");

            // Handle controls
            if (gamepad1.a) {
                currentPointIndex = (currentPointIndex + 1) % CALIBRATION_POINTS.length;
                sleep(250); // Debounce
            } else if (gamepad1.b) {
                currentPointIndex = (currentPointIndex - 1 + CALIBRATION_POINTS.length) % CALIBRATION_POINTS.length;
                sleep(250); // Debounce
            }

            telemetry.update();
        }
    }

    private String formatPose(Pose2d pose) {
        return String.format("X: %.1f, Y: %.1f, Heading: %.1f°",
                pose.position.x,
                pose.position.y,
                Math.toDegrees(pose.heading.toDouble()));
    }
}