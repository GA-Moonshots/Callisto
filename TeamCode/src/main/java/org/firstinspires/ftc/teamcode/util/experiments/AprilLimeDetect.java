package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
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
 *
 * It uses a limited set of buttons (B, X, and D-Pad left/right/down)
 * to control all calibration functions.
 *
 * Features:
 * - Real-time visualization of AprilTag positions
 * - Calibration of position and orientation offsets
 * - Movement testing to verify accuracy
 * - Dashboard visualization
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

    // Configurable parameters
    private final boolean DEBUG_MODE = true;
    private final double MAX_POSE_CHANGE_INCHES = 2.0;
    private final double MAX_HEADING_CHANGE_DEGREES = 5.0;
    private final int TAG_FRESHNESS_MS = 500;

    // Adjustment step sizes
    private final double POSITION_ADJUST_STEP = 0.01; // meters
    private final double HEADING_ADJUST_STEP = 1.0;   // degrees

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
        telemetry.addData("AprilTag Detect", "Started");

        // Reset test state
        readyForMovementTest = false;
        hasMovementTestCompleted = false;
        preMovePose = null;
        postMovePose = null;

        // Initialize adjustment mode
        currentMode = AdjustmentMode.X_OFFSET;

        // Reset button states
        wasRightPressed = false;
        wasLeftPressed = false;
        wasDownPressed = false;
        wasBPressed = false;
        wasXPressed = false;
    }

    @Override
    public void execute() {
        // Get the Limelight result and prepare dashboard packet
        LLResult result = sensors.getLLResult();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Testing Mode", "AprilTag Calibration");

        // Process AprilTag detection if valid
        if(result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                telemetry.addLine("⭐ AprilTag Detected ⭐");
            }

            // Extract tag position data
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Save detection data
                lastBotpose = botpose;
                lastRoadRunnerPose = robot.mecanum.pose;
                lastTagDetectionTime = System.currentTimeMillis();

                // Extract and process raw position data
                double rawX = botpose.getPosition().x;
                double rawY = botpose.getPosition().y;
                double rawHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Apply calibration offsets
                double adjustedX = rawX + xOffsetMeters;
                double adjustedY = rawY + yOffsetMeters;
                double adjustedHeading = rawHeading + headingOffsetDegrees;
                if(adjustedHeading < 0) adjustedHeading += 360;

                // Convert to inches for RoadRunner
                double roadrunnerX = adjustedX * 39.3701;
                double roadrunnerY = adjustedY * 39.3701;
                double roadrunnerHeading = Math.toRadians(adjustedHeading);

                // Display camera view data
                telemetry.addLine("⭐ AprilTag Detected ⭐");
                telemetry.addLine("----- Camera View -----");
                telemetry.addData("Horizontal Offset (tx)", String.format("%.2f", tx));
                telemetry.addData("Vertical Offset (ty)", String.format("%.2f", ty));
                telemetry.addData("Target Area (ta)", String.format("%.2f", ta));

                // Display raw position data
                telemetry.addLine("----- Raw Limelight Position (meters) -----");
                telemetry.addData("Raw X", String.format("%.3f m", rawX));
                telemetry.addData("Raw Y", String.format("%.3f m", rawY));
                telemetry.addData("Raw Heading", String.format("%.1f°", rawHeading));

                // Display adjusted position data
                telemetry.addLine("----- Adjusted Position (meters) -----");
                telemetry.addData("Adjusted X", String.format("%.3f m", adjustedX));
                telemetry.addData("Adjusted Y", String.format("%.3f m", adjustedY));
                telemetry.addData("Adjusted Heading", String.format("%.1f°", adjustedHeading));

                // Display RoadRunner position data
                telemetry.addLine("----- RoadRunner Position (inches) -----");
                telemetry.addData("RoadRunner X", String.format("%.1f in", roadrunnerX));
                telemetry.addData("RoadRunner Y", String.format("%.1f in", roadrunnerY));
                telemetry.addData("RoadRunner Heading", String.format("%.1f°", Math.toDegrees(roadrunnerHeading)));

                // Display current calibration offsets with active mode highlighted
                telemetry.addLine("----- Current Offsets -----");
                telemetry.addData("X Offset" + (currentMode == AdjustmentMode.X_OFFSET ? " ▶" : ""),
                        String.format("%.3f m", xOffsetMeters));
                telemetry.addData("Y Offset" + (currentMode == AdjustmentMode.Y_OFFSET ? " ▶" : ""),
                        String.format("%.3f m", yOffsetMeters));
                telemetry.addData("Heading Offset" + (currentMode == AdjustmentMode.HEADING_OFFSET ? " ▶" : ""),
                        String.format("%.1f°", headingOffsetDegrees));

                // Compare with current RoadRunner pose
                telemetry.addLine("----- Current RoadRunner Pose -----");
                telemetry.addData("Current X", String.format("%.1f in", robot.mecanum.pose.position.x));
                telemetry.addData("Current Y", String.format("%.1f in", robot.mecanum.pose.position.y));
                telemetry.addData("Current Heading", String.format("%.1f°",
                        Math.toDegrees(robot.mecanum.pose.heading.toDouble())));

                // Calculate differences between detected and current position
                double xDiff = roadrunnerX - robot.mecanum.pose.position.x;
                double yDiff = roadrunnerY - robot.mecanum.pose.position.y;
                double headingDiff = normalizeAngleDegrees(
                        Math.toDegrees(roadrunnerHeading) -
                                Math.toDegrees(robot.mecanum.pose.heading.toDouble()));

                telemetry.addLine("----- Position Differences -----");
                telemetry.addData("X Difference", String.format("%.1f in", xDiff));
                telemetry.addData("Y Difference", String.format("%.1f in", yDiff));
                telemetry.addData("Heading Difference", String.format("%.1f°", headingDiff));

                // Visualize on dashboard
                packet.fieldOverlay()
                        .setStroke("#FF0000")
                        .strokeCircle(roadrunnerX, roadrunnerY, 5) // Red circle for AprilTag position
                        .setStroke("#0000FF")
                        .strokeCircle(robot.mecanum.pose.position.x, robot.mecanum.pose.position.y, 5); // Blue for current

                dashboard.sendTelemetryPacket(packet);

                // Handle movement test logic - record positions before and after movement
                if (readyForMovementTest && preMovePose == null) {
                    // Record the starting pose before movement
                    preMovePose = new Pose2d(
                            roadrunnerX, roadrunnerY, roadrunnerHeading);
                    telemetry.addLine("⚠ MOVEMENT TEST: Starting Position Recorded ⚠");
                }

                if (hasMovementTestCompleted && postMovePose == null) {
                    // Record the ending pose after movement
                    postMovePose = new Pose2d(
                            roadrunnerX, roadrunnerY, roadrunnerHeading);
                    telemetry.addLine("⚠ MOVEMENT TEST: Ending Position Recorded ⚠");

                    // Calculate movement detected by AprilTag
                    if (preMovePose != null) {
                        double aprilTagMoveX = postMovePose.position.x - preMovePose.position.x;
                        double aprilTagMoveY = postMovePose.position.y - preMovePose.position.y;
                        double aprilTagRotate = Math.toDegrees(postMovePose.heading.toDouble() -
                                preMovePose.heading.toDouble());

                        telemetry.addLine("----- Movement Test Results -----");
                        telemetry.addData("AprilTag-Detected X Movement",
                                String.format("%.1f in", aprilTagMoveX));
                        telemetry.addData("AprilTag-Detected Y Movement",
                                String.format("%.1f in", aprilTagMoveY));
                        telemetry.addData("AprilTag-Detected Rotation",
                                String.format("%.1f°", aprilTagRotate));
                    }
                }
            }
        } else {
            // Display status when no tag is detected
            telemetry.addData("Status", "No AprilTag detected");
            telemetry.addData("Last detection", lastTagDetectionTime > 0 ?
                    (System.currentTimeMillis() - lastTagDetectionTime) + "ms ago" : "Never");
            if (lastTagDetectionTime > 0 && lastBotpose != null) {
                telemetry.addData("Last Tag ID", tagID);
            }
        }

        // Display movement test status
        if (!readyForMovementTest) {
            telemetry.addLine("\n🔵 Press B to start movement test");
        } else if (preMovePose == null) {
            telemetry.addLine("\n🟡 Position yourself in view of AprilTag");
        } else if (!hasMovementTestCompleted) {
            telemetry.addLine("\n🟢 Press X to make test movement");
        } else if (postMovePose == null) {
            telemetry.addLine("\n🟠 Waiting for AprilTag detection after movement");
        } else {
            telemetry.addLine("\n✅ Movement test complete!");
            telemetry.addLine("Press B again to reset test");
        }

        // Display button controls for adjustments
        telemetry.addLine("\n----- CONTROLS (LIMITED BUTTONS) -----");
        telemetry.addData("D-PAD RIGHT", "Cycle adjustment mode");
        telemetry.addData("D-PAD LEFT", "Increase selected value");
        telemetry.addData("D-PAD DOWN", "Decrease selected value");
        telemetry.addData("B Button", "Start/Reset movement test");
        telemetry.addData("X Button", "Execute test movement");
        telemetry.addData("Current Mode", getCurrentModeString());

        // ===== LIMITED BUTTON CONTROLS =====
        // Check D-Pad RIGHT to cycle between adjustment modes
        boolean rightPressed = robot.opMode.gamepad1.dpad_right;
        if (rightPressed && !wasRightPressed) {
            // Cycle to next adjustment mode
            currentMode = AdjustmentMode.values()[
                    (currentMode.ordinal() + 1) % AdjustmentMode.values().length];
            telemetry.addData("Mode Changed", getCurrentModeString());
            sleep(50); // Small debounce
        }
        wasRightPressed = rightPressed;

        // D-Pad LEFT to increase current value
        boolean leftPressed = robot.opMode.gamepad1.dpad_left;
        if (leftPressed && !wasLeftPressed) {
            // Increase value based on current mode
            switch (currentMode) {
                case X_OFFSET:
                    xOffsetMeters += POSITION_ADJUST_STEP;
                    telemetry.addData("X Offset Increased",
                            String.format("%.3f m", xOffsetMeters));
                    break;
                case Y_OFFSET:
                    yOffsetMeters += POSITION_ADJUST_STEP;
                    telemetry.addData("Y Offset Increased",
                            String.format("%.3f m", yOffsetMeters));
                    break;
                case HEADING_OFFSET:
                    headingOffsetDegrees += HEADING_ADJUST_STEP;
                    telemetry.addData("Heading Offset Increased",
                            String.format("%.1f°", headingOffsetDegrees));
                    break;
            }
            sleep(50); // Small debounce
        }
        wasLeftPressed = leftPressed;

        // D-Pad DOWN to decrease current value
        boolean downPressed = robot.opMode.gamepad1.dpad_down;
        if (downPressed && !wasDownPressed) {
            // Decrease value based on current mode
            switch (currentMode) {
                case X_OFFSET:
                    xOffsetMeters -= POSITION_ADJUST_STEP;
                    telemetry.addData("X Offset Decreased",
                            String.format("%.3f m", xOffsetMeters));
                    break;
                case Y_OFFSET:
                    yOffsetMeters -= POSITION_ADJUST_STEP;
                    telemetry.addData("Y Offset Decreased",
                            String.format("%.3f m", yOffsetMeters));
                    break;
                case HEADING_OFFSET:
                    headingOffsetDegrees -= HEADING_ADJUST_STEP;
                    telemetry.addData("Heading Offset Decreased",
                            String.format("%.1f°", headingOffsetDegrees));
                    break;
            }
            sleep(50); // Small debounce
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
                telemetry.addData("Movement Test", "Reset");
            } else {
                // Start new test
                readyForMovementTest = true;
                preMovePose = null;
                hasMovementTestCompleted = false;
                telemetry.addData("Movement Test", "Ready to start");
            }
            sleep(300); // Debounce
        }
        wasBPressed = bPressed;

        // X button to execute test movement
        boolean xPressed = robot.opMode.gamepad1.x;
        if (xPressed && !wasXPressed && readyForMovementTest &&
                preMovePose != null && !hasMovementTestCompleted) {
            // Execute the test movement
            hasMovementTestCompleted = true;
            telemetry.addData("Movement Test", "Moving...");
            performTestMovement();
            sleep(300); // Debounce
        }
        wasXPressed = xPressed;
    }

    /**
     * Returns a string representation of the current adjustment mode
     */
    private String getCurrentModeString() {
        switch (currentMode) {
            case X_OFFSET: return "X Offset Adjustment";
            case Y_OFFSET: return "Y Offset Adjustment";
            case HEADING_OFFSET: return "Heading Offset Adjustment";
            default: return "Unknown";
        }
    }

    /**
     * Executes a simple forward movement to test localization accuracy
     */
    private void performTestMovement() {
        new SequentialCommandGroup(
                new InstantCommand(() -> telemetry.addData("Test Movement", "Starting...")),
                // Move forward 10 inches
                new StrafeToPose(robot,
                        new Pose2d(
                                robot.mecanum.pose.position.x + 10,
                                robot.mecanum.pose.position.y,
                                robot.mecanum.pose.heading.toDouble()),
                        2.0), // 2 second timeout
                new InstantCommand(() -> telemetry.addData("Test Movement", "Completed"))
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
        telemetry.addData("AprilTag Detect", "Ended");

        // Provide summary of calibration findings
        if (lastBotpose != null) {
            telemetry.addLine("\n----- Final Calibration Values -----");
            telemetry.addData("X Offset", String.format("%.3f m", xOffsetMeters));
            telemetry.addData("Y Offset", String.format("%.3f m", yOffsetMeters));
            telemetry.addData("Heading Offset", String.format("%.1f°", headingOffsetDegrees));
            telemetry.addLine("\nAdd these values to your SensorPackage.updatePose() method");
        }
    }
}