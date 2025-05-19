package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.List;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.util.Timing;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.commands.StrafeToPose;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;

/**
 * AprilLimeDetect - AprilTag Calibration and Testing Command
 *
 * This command provides an interface for testing and calibrating
 * the AprilTag detection system using the Limelight camera.
 */
public class AprilLimeDetect extends CommandBase {
    // Robot and subsystem references
    private final Callisto robot;
    private final SensorPackage sensors;
    private Timing.Timer timer;
    private FtcDashboard dashboard;
    private MultipleTelemetry telemetry;

    // Movement test state tracking
    private boolean readyForMovementTest = false;
    private Pose2d preMovePose = null;
    private Pose2d postMovePose = null;
    private boolean hasMovementTestCompleted = false;

    // Calibration values
    private double xOffsetMeters = 0.0;
    private double yOffsetMeters = 0.0;
    private double headingOffsetDegrees = 0.0;

    // Current adjustment mode (cycles through with D-Pad right)
    private enum AdjustmentMode { X_OFFSET, Y_OFFSET, HEADING_OFFSET }
    private AdjustmentMode currentMode = AdjustmentMode.X_OFFSET;

    // Button state tracking to prevent multiple triggers
    private boolean wasRightPressed = false;
    private boolean wasLeftPressed = false;
    private boolean wasDownPressed = false;
    private boolean wasBPressed = false;
    private boolean wasXPressed = false;

    // Tracking variables for detected tags
    private Pose3D lastBotpose = null;
    private Pose2d lastRoadRunnerPose = null;
    private long lastTagDetectionTime = 0;
    private int tagID = -1;

    // Adjustment step sizes
    private final double POSITION_ADJUST_STEP = 0.01; // meters
    private final double HEADING_ADJUST_STEP = 1.0;   // degrees

    // Visualization settings
    private final double HEADING_LINE_LENGTH = 8.0; // inches
    private final String APRILTAG_COLOR = "#FF0000"; // Red
    private final String ROADRUNNER_COLOR = "#0000FF"; // Blue

    /**
     * Initializes the AprilTag detection and calibration command.
     *
     * @param robot The robot container with access to subsystems
     */
    public AprilLimeDetect(Callisto robot) {
        this.robot = robot;
        this.sensors = robot.sensors;
        this.dashboard = FtcDashboard.getInstance();
        timer = new Timing.Timer((long) 0.5);
        addRequirements(sensors);
    }

    @Override
    public void initialize() {
        // Reset state
        timer.start();
        sensors.enableAprilTagTracking();
        telemetry = new MultipleTelemetry(robot.telemetry, dashboard.getTelemetry());
        telemetry.addData("Status", "AprilTag Detect Started");

        // Reset test state
        readyForMovementTest = false;
        hasMovementTestCompleted = false;
        preMovePose = null;
        postMovePose = null;

        // Initialize adjustment mode
        currentMode = AdjustmentMode.X_OFFSET;

        // Reset button states
        wasRightPressed = wasLeftPressed = wasDownPressed = wasBPressed = wasXPressed = false;
    }

    @Override
    public void execute() {
        // Get the Limelight result and prepare dashboard packet
        LLResult result = sensors.getLLResult();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Mode", "AprilTag Calibration");

        // Display current calibration values (always shown)
        telemetry.addLine("==== Current Offsets ====");
        telemetry.addData("X Offset" + (currentMode == AdjustmentMode.X_OFFSET ? " ▶" : ""),
                String.format("%.3f m", xOffsetMeters));
        telemetry.addData("Y Offset" + (currentMode == AdjustmentMode.Y_OFFSET ? " ▶" : ""),
                String.format("%.3f m", yOffsetMeters));
        telemetry.addData("Heading Offset" + (currentMode == AdjustmentMode.HEADING_OFFSET ? " ▶" : ""),
                String.format("%.1f°", headingOffsetDegrees));

        // Process AprilTag detection if valid
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Save detection data
                lastBotpose = botpose;
                lastRoadRunnerPose = robot.mecanum.pose;
                lastTagDetectionTime = System.currentTimeMillis();

                // Get tag ID if available
                if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                    try {
                        // Based on Limelight documentation, FiducialResult objects have getFiducialId() method
                        List<LLResultTypes.FiducialResult> fiducialResults = result.getFiducialResults();

                        for (LLResultTypes.FiducialResult fr : fiducialResults) {
                            // Get the ID directly from the FiducialResult object
                            tagID = fr.getFiducialId();
                            telemetry.addData("Tag ID", tagID);

                            // We only need the first tag for this test
                            break;
                        }
                    } catch (Exception e) {
                        telemetry.addData("Tag ID Error", e.getMessage());
                    }
                }

                // Extract and process raw position data
                double rawX = botpose.getPosition().x;
                double rawY = botpose.getPosition().y;
                double rawHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Apply calibration offsets
                double adjustedX = rawX + xOffsetMeters;
                double adjustedY = rawY + yOffsetMeters;
                double adjustedHeading = rawHeading + headingOffsetDegrees;
                if (adjustedHeading < 0) adjustedHeading += 360;

                // Convert to inches for RoadRunner
                double roadrunnerX = adjustedX * 39.3701;
                double roadrunnerY = adjustedY * 39.3701;
                double roadrunnerHeading = Math.toRadians(adjustedHeading);

                // Create pose objects for comparison
                Pose2d aprilTagPose = new Pose2d(roadrunnerX, roadrunnerY, Math.toRadians(adjustedHeading));
                Pose2d currentPose = robot.mecanum.pose;

                // Display AprilTag vs RoadRunner position comparison
                telemetry.addLine("\n==== Position Info ====");
                telemetry.addData("AprilTag", String.format("[%.1f, %.1f] %.1f°",
                        aprilTagPose.position.x, aprilTagPose.position.y,
                        Math.toDegrees(aprilTagPose.heading.toDouble())));
                telemetry.addData("RoadRunner", String.format("[%.1f, %.1f] %.1f°",
                        currentPose.position.x, currentPose.position.y,
                        Math.toDegrees(currentPose.heading.toDouble())));

                // Calculate differences
                double xDiff = aprilTagPose.position.x - currentPose.position.x;
                double yDiff = aprilTagPose.position.y - currentPose.position.y;
                double headingDiff = normalizeAngleDegrees(
                        Math.toDegrees(aprilTagPose.heading.toDouble()) -
                                Math.toDegrees(currentPose.heading.toDouble()));

                telemetry.addData("Difference", String.format("[%.1f, %.1f] %.1f°",
                        xDiff, yDiff, headingDiff));

                // Draw AprilTag pose with heading indicator
                drawPoseWithHeading(packet.fieldOverlay(), aprilTagPose, APRILTAG_COLOR);

                // Draw current RoadRunner pose with heading indicator
                drawPoseWithHeading(packet.fieldOverlay(), currentPose, ROADRUNNER_COLOR);

                // Add a legend
                packet.fieldOverlay()
                        .setStroke(APRILTAG_COLOR)
                        .strokeLine(10, 10, 30, 10)
                        .setStroke(ROADRUNNER_COLOR)
                        .strokeLine(10, 25, 30, 25);

                dashboard.sendTelemetryPacket(packet);

                // Handle movement test logic
                handleMovementTest(aprilTagPose);
            }
        } else {
            // Display status when no tag is detected
            telemetry.addData("Status", "No AprilTag detected");
            if (lastTagDetectionTime > 0) {
                long timeAgo = System.currentTimeMillis() - lastTagDetectionTime;
                telemetry.addData("Last detection", timeAgo < 5000 ?
                        timeAgo + "ms ago" : "> 5s ago");
            }

            // Draw only current RoadRunner pose
            drawPoseWithHeading(packet.fieldOverlay(), robot.mecanum.pose, ROADRUNNER_COLOR);
            dashboard.sendTelemetryPacket(packet);
        }

        // Display movement test status
        displayMovementTestStatus();

        // Display button controls (simplified)
        telemetry.addLine("\n==== CONTROLS ====");
        telemetry.addData("RIGHT", "Next mode | LEFT", "Increase | DOWN: Decrease");
        telemetry.addData("B", "Start/Reset test | X", "Execute movement");

        // Handle button inputs
        handleButtonInputs();
    }

    /**
     * Draws a pose with heading line on the field overlay
     */
    private void drawPoseWithHeading(Object fieldOverlay, Pose2d pose, String color) {
        // Cast to the correct type (the method parameter is kept as Object to avoid compile errors if the API changes)
        com.acmerobotics.dashboard.canvas.Canvas canvas = (com.acmerobotics.dashboard.canvas.Canvas) fieldOverlay;

        // Draw robot position circle
        canvas.setStroke(color);
        canvas.strokeCircle(pose.position.x, pose.position.y, 5);

        // Calculate end point of heading line using trig
        Vector2d headingVector = new Vector2d(
                Math.cos(pose.heading.toDouble()) * HEADING_LINE_LENGTH,
                Math.sin(pose.heading.toDouble()) * HEADING_LINE_LENGTH
        );

        // Draw heading line
        canvas.setStrokeWidth(2);
        canvas.strokeLine(
                pose.position.x,
                pose.position.y,
                pose.position.x + headingVector.x,
                pose.position.y + headingVector.y
        );

        // Draw small arrow at the end of the heading line
        double arrowSize = 2.0;
        double arrowAngle = Math.PI / 6; // 30 degrees

        double headingAngle = pose.heading.toDouble();
        Vector2d arrowLeft = new Vector2d(
                headingVector.x - arrowSize * Math.cos(headingAngle + Math.PI - arrowAngle),
                headingVector.y - arrowSize * Math.sin(headingAngle + Math.PI - arrowAngle)
        );

        Vector2d arrowRight = new Vector2d(
                headingVector.x - arrowSize * Math.cos(headingAngle + Math.PI + arrowAngle),
                headingVector.y - arrowSize * Math.sin(headingAngle + Math.PI + arrowAngle)
        );

        canvas.strokeLine(
                pose.position.x + headingVector.x,
                pose.position.y + headingVector.y,
                pose.position.x + arrowLeft.x,
                pose.position.y + arrowLeft.y
        );

        canvas.strokeLine(
                pose.position.x + headingVector.x,
                pose.position.y + headingVector.y,
                pose.position.x + arrowRight.x,
                pose.position.y + arrowRight.y
        );
    }

    /**
     * Handles the movement test logic and displays status
     */
    private void handleMovementTest(Pose2d aprilTagPose) {
        if (readyForMovementTest && preMovePose == null) {
            // Record the starting pose before movement
            preMovePose = aprilTagPose;
            telemetry.addData("Test", "Starting position recorded");
        }

        if (hasMovementTestCompleted && postMovePose == null) {
            // Record the ending pose after movement
            postMovePose = aprilTagPose;
            telemetry.addData("Test", "Ending position recorded");

            // Calculate movement detected by AprilTag
            if (preMovePose != null) {
                double moveX = postMovePose.position.x - preMovePose.position.x;
                double moveY = postMovePose.position.y - preMovePose.position.y;
                double rotate = Math.toDegrees(postMovePose.heading.toDouble() -
                        preMovePose.heading.toDouble());

                telemetry.addLine("\n==== Movement Results ====");
                telemetry.addData("Moved", String.format("[%.1f, %.1f] %.1f°",
                        moveX, moveY, rotate));
            }
        }
    }

    /**
     * Displays the current movement test status
     */
    private void displayMovementTestStatus() {
        telemetry.addLine("\n==== Test Status ====");
        if (!readyForMovementTest) {
            telemetry.addData("Status", "⚪ Press B to start test");
        } else if (preMovePose == null) {
            telemetry.addData("Status", "🟡 Position at AprilTag");
        } else if (!hasMovementTestCompleted) {
            telemetry.addData("Status", "🟢 Press X to move");
        } else if (postMovePose == null) {
            telemetry.addData("Status", "🟠 Waiting for detection");
        } else {
            telemetry.addData("Status", "✅ Test complete (B to reset)");
        }
    }

    /**
     * Handles all button inputs with debouncing
     */
    private void handleButtonInputs() {
        // D-Pad RIGHT to cycle between adjustment modes
        boolean rightPressed = robot.opMode.gamepad1.dpad_right;
        if (rightPressed && !wasRightPressed) {
            currentMode = AdjustmentMode.values()[
                    (currentMode.ordinal() + 1) % AdjustmentMode.values().length];
            sleep(50);
        }
        wasRightPressed = rightPressed;

        // D-Pad LEFT to increase current value
        boolean leftPressed = robot.opMode.gamepad1.dpad_left;
        if (leftPressed && !wasLeftPressed) {
            switch (currentMode) {
                case X_OFFSET:
                    xOffsetMeters += POSITION_ADJUST_STEP;
                    break;
                case Y_OFFSET:
                    yOffsetMeters += POSITION_ADJUST_STEP;
                    break;
                case HEADING_OFFSET:
                    headingOffsetDegrees += HEADING_ADJUST_STEP;
                    break;
            }
            sleep(50);
        }
        wasLeftPressed = leftPressed;

        // D-Pad DOWN to decrease current value
        boolean downPressed = robot.opMode.gamepad1.dpad_down;
        if (downPressed && !wasDownPressed) {
            switch (currentMode) {
                case X_OFFSET:
                    xOffsetMeters -= POSITION_ADJUST_STEP;
                    break;
                case Y_OFFSET:
                    yOffsetMeters -= POSITION_ADJUST_STEP;
                    break;
                case HEADING_OFFSET:
                    headingOffsetDegrees -= HEADING_ADJUST_STEP;
                    break;
            }
            sleep(50);
        }
        wasDownPressed = downPressed;

        // B button to start/reset movement test
        boolean bPressed = robot.opMode.gamepad1.b;
        if (bPressed && !wasBPressed) {
            if (postMovePose != null) {
                // Reset the test if completed
                readyForMovementTest = false;
                preMovePose = null;
                postMovePose = null;
                hasMovementTestCompleted = false;
            } else {
                // Start new test
                readyForMovementTest = true;
                preMovePose = null;
                hasMovementTestCompleted = false;
            }
            sleep(300);
        }
        wasBPressed = bPressed;

        // X button to execute test movement
        boolean xPressed = robot.opMode.gamepad1.x;
        if (xPressed && !wasXPressed && readyForMovementTest &&
                preMovePose != null && !hasMovementTestCompleted) {
            // Execute the test movement
            hasMovementTestCompleted = true;
            performTestMovement();
            sleep(300);
        }
        wasXPressed = xPressed;
    }

    /**
     * Returns a string representation of the current adjustment mode
     */
    private String getCurrentModeString() {
        switch (currentMode) {
            case X_OFFSET: return "X Offset";
            case Y_OFFSET: return "Y Offset";
            case HEADING_OFFSET: return "Heading Offset";
            default: return "Unknown";
        }
    }

    /**
     * Executes a simple forward movement to test localization accuracy
     */
    private void performTestMovement() {
        new SequentialCommandGroup(
                new InstantCommand(() -> telemetry.addData("Movement", "Starting...")),
                // Move forward 10 inches
                new StrafeToPose(robot,
                        new Pose2d(
                                robot.mecanum.pose.position.x + 10,
                                robot.mecanum.pose.position.y,
                                robot.mecanum.pose.heading.toDouble()),
                        2.0), // 2 second timeout
                new InstantCommand(() -> telemetry.addData("Movement", "Completed"))
        ).schedule();
    }

    /**
     * Normalizes an angle to the range [-180, 180] degrees
     */
    private double normalizeAngleDegrees(double degrees) {
        double normalized = degrees;
        while (normalized > 180) normalized -= 360;
        while (normalized < -180) normalized += 360;
        return normalized;
    }

    /**
     * Simple sleep helper for button debouncing
     */
    private void sleep(long ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    public boolean isFinished() {
        return false; // Run continuously until canceled
    }

    @Override
    public void end(boolean interrupted) {
        telemetry.addData("Status", "AprilTag Detect Ended");

        // Provide summary of calibration values
        telemetry.addLine("\n==== Final Calibration Values ====");
        telemetry.addData("X Offset", String.format("%.3f m", xOffsetMeters));
        telemetry.addData("Y Offset", String.format("%.3f m", yOffsetMeters));
        telemetry.addData("Heading Offset", String.format("%.1f°", headingOffsetDegrees));
    }
}