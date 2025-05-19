package org.firstinspires.ftc.teamcode.util.experiments;

import com.acmerobotics.dashboard.FtcDashboard;
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


public class AprilLimeDetect extends CommandBase {
    private final Callisto robot;
    private final SensorPackage sensors;
    private Timing.Timer timer;
    private FtcDashboard dashboard;

    // Movement test variables
    private boolean readyForMovementTest = false;
    private Pose2d preMovePose = null;
    private Pose2d postMovePose = null;
    private boolean hasMovementTestCompleted = false;

    // Offset calibration variables
    private double xOffsetMeters = 0.0;
    private double yOffsetMeters = 0.0;
    private double headingOffsetDegrees = 0.0;

    // Tracking variables
    private Pose3D lastBotpose = null;
    private Pose2d lastRoadRunnerPose = null;
    private long lastTagDetectionTime = 0;
    private int tagID = -1;

    // Configurable parameters
    private final boolean DEBUG_MODE = true;
    private final double MAX_POSE_CHANGE_INCHES = 2.0; // Maximum change to trust
    private final double MAX_HEADING_CHANGE_DEGREES = 5.0;
    private final int TAG_FRESHNESS_MS = 500; // How recent a tag detection needs to be

    public AprilLimeDetect(Callisto robot) {
        this.robot = robot;
        this.sensors = robot.sensors;
        this.dashboard = FtcDashboard.getInstance();
        timer = new Timing.Timer((long) 0.5); // Short timer for telemetry
        addRequirements(sensors);
    }

    @Override
    public void initialize() {
        timer.start();
        sensors.enableAprilTagTracking();
        robot.telemetry.addData("AprilTag Detect", "Started");
        readyForMovementTest = false;
        hasMovementTestCompleted = false;
        preMovePose = null;
        postMovePose = null;
    }

    @Override
    public void execute() {
        // Get the Limelight result
        LLResult result = sensors.getLLResult();

        // Create a dashboard packet for visualization
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Testing Mode", "AprilTag Calibration");

        if(result != null && result.isValid()) {
            if (result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                robot.telemetry.addLine("⭐ AprilTag Detected ⭐");
            }

            double tx = result.getTx(); // Tag position relative to camera (X)
            double ty = result.getTy(); // Tag position relative to camera (Y)
            double ta = result.getTa(); // Tag area (size in image)

            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                // Save the last detected position
                lastBotpose = botpose;
                lastRoadRunnerPose = robot.mecanum.pose;
                lastTagDetectionTime = System.currentTimeMillis();

                // Extract the raw positions
                double rawX = botpose.getPosition().x;
                double rawY = botpose.getPosition().y;
                double rawHeading = botpose.getOrientation().getYaw(AngleUnit.DEGREES);

                // Apply the current offsets
                double adjustedX = rawX + xOffsetMeters;
                double adjustedY = rawY + yOffsetMeters;
                double adjustedHeading = rawHeading + headingOffsetDegrees;
                if(adjustedHeading < 0) adjustedHeading += 360;

                // Convert to inches for RoadRunner
                double roadrunnerX = adjustedX * 39.3701;
                double roadrunnerY = adjustedY * 39.3701;
                double roadrunnerHeading = Math.toRadians(adjustedHeading);

                // Display all values
                robot.telemetry.addLine("⭐ AprilTag Detected (ID: " + tagID + ") ⭐");
                robot.telemetry.addLine("----- Camera View -----");
                robot.telemetry.addData("Horizontal Offset (tx)", String.format("%.2f", tx));
                robot.telemetry.addData("Vertical Offset (ty)", String.format("%.2f", ty));
                robot.telemetry.addData("Target Area (ta)", String.format("%.2f", ta));

                robot.telemetry.addLine("----- Raw Limelight Position (meters) -----");
                robot.telemetry.addData("Raw X", String.format("%.3f m", rawX));
                robot.telemetry.addData("Raw Y", String.format("%.3f m", rawY));
                robot.telemetry.addData("Raw Heading", String.format("%.1f°", rawHeading));

                robot.telemetry.addLine("----- Adjusted Position (meters) -----");
                robot.telemetry.addData("Adjusted X", String.format("%.3f m", adjustedX));
                robot.telemetry.addData("Adjusted Y", String.format("%.3f m", adjustedY));
                robot.telemetry.addData("Adjusted Heading", String.format("%.1f°", adjustedHeading));

                robot.telemetry.addLine("----- RoadRunner Position (inches) -----");
                robot.telemetry.addData("RoadRunner X", String.format("%.1f in", roadrunnerX));
                robot.telemetry.addData("RoadRunner Y", String.format("%.1f in", roadrunnerY));
                robot.telemetry.addData("RoadRunner Heading", String.format("%.1f°", Math.toDegrees(roadrunnerHeading)));

                robot.telemetry.addLine("----- Current Offsets -----");
                robot.telemetry.addData("X Offset", String.format("%.3f m", xOffsetMeters));
                robot.telemetry.addData("Y Offset", String.format("%.3f m", yOffsetMeters));
                robot.telemetry.addData("Heading Offset", String.format("%.1f°", headingOffsetDegrees));

                // Compare with current RoadRunner pose
                robot.telemetry.addLine("----- Current RoadRunner Pose -----");
                robot.telemetry.addData("Current X", String.format("%.1f in", robot.mecanum.pose.position.x));
                robot.telemetry.addData("Current Y", String.format("%.1f in", robot.mecanum.pose.position.y));
                robot.telemetry.addData("Current Heading", String.format("%.1f°",
                        Math.toDegrees(robot.mecanum.pose.heading.toDouble())));

                // Calculate differences
                double xDiff = roadrunnerX - robot.mecanum.pose.position.x;
                double yDiff = roadrunnerY - robot.mecanum.pose.position.y;
                double headingDiff = normalizeAngleDegrees(
                        Math.toDegrees(roadrunnerHeading) -
                                Math.toDegrees(robot.mecanum.pose.heading.toDouble()));

                robot.telemetry.addLine("----- Position Differences -----");
                robot.telemetry.addData("X Difference", String.format("%.1f in", xDiff));
                robot.telemetry.addData("Y Difference", String.format("%.1f in", yDiff));
                robot.telemetry.addData("Heading Difference", String.format("%.1f°", headingDiff));

                // Visualize on dashboard
                packet.fieldOverlay()
                        .setStroke("#FF0000")
                        .strokeCircle(roadrunnerX, roadrunnerY, 5) // Red circle for AprilTag position
                        .setStroke("#0000FF")
                        .strokeCircle(robot.mecanum.pose.position.x, robot.mecanum.pose.position.y, 5); // Blue for current

                dashboard.sendTelemetryPacket(packet);

                // Movement test logic
                if (readyForMovementTest && preMovePose == null) {
                    // Record the starting pose before movement
                    preMovePose = new Pose2d(
                            roadrunnerX, roadrunnerY, roadrunnerHeading);
                    robot.telemetry.addLine("⚠ MOVEMENT TEST: Starting Position Recorded ⚠");
                }

                if (hasMovementTestCompleted && postMovePose == null) {
                    // Record the ending pose after movement
                    postMovePose = new Pose2d(
                            roadrunnerX, roadrunnerY, roadrunnerHeading);
                    robot.telemetry.addLine("⚠ MOVEMENT TEST: Ending Position Recorded ⚠");

                    // Calculate movement detected by AprilTag
                    if (preMovePose != null) {
                        double aprilTagMoveX = postMovePose.position.x - preMovePose.position.x;
                        double aprilTagMoveY = postMovePose.position.y - preMovePose.position.y;
                        double aprilTagRotate = Math.toDegrees(postMovePose.heading.toDouble() -
                                preMovePose.heading.toDouble());

                        robot.telemetry.addLine("----- Movement Test Results -----");
                        robot.telemetry.addData("AprilTag-Detected X Movement",
                                String.format("%.1f in", aprilTagMoveX));
                        robot.telemetry.addData("AprilTag-Detected Y Movement",
                                String.format("%.1f in", aprilTagMoveY));
                        robot.telemetry.addData("AprilTag-Detected Rotation",
                                String.format("%.1f°", aprilTagRotate));

                        // We could also compare with what RoadRunner thinks happened
                        // This could be added
                    }
                }
            }
        } else {
            robot.telemetry.addData("Status", "No AprilTag detected");
            robot.telemetry.addData("Last detection", lastTagDetectionTime > 0 ?
                    (System.currentTimeMillis() - lastTagDetectionTime) + "ms ago" : "Never");
            if (lastTagDetectionTime > 0 && lastBotpose != null) {
                robot.telemetry.addData("Last Tag ID", tagID);
            }
        }

        // Movement test status
        if (!readyForMovementTest) {
            robot.telemetry.addLine("\n🔵 Press B to start movement test");
        } else if (preMovePose == null) {
            robot.telemetry.addLine("\n🟡 Position yourself in view of AprilTag");
        } else if (!hasMovementTestCompleted) {
            robot.telemetry.addLine("\n🟢 Press X to make test movement");
        } else if (postMovePose == null) {
            robot.telemetry.addLine("\n🟠 Waiting for AprilTag detection after movement");
        } else {
            robot.telemetry.addLine("\n✅ Movement test complete!");
            robot.telemetry.addLine("Press Y to reset test");
        }

        // Offset adjustment controls
        robot.telemetry.addLine("\n----- Offset Adjustments -----");
        robot.telemetry.addData("DPAD UP/DOWN", "Adjust Y Offset (±0.01m)");
        robot.telemetry.addData("DPAD LEFT/RIGHT", "Adjust X Offset (±0.01m)");
        robot.telemetry.addData("LEFT/RIGHT BUMPER", "Adjust Heading Offset (±1°)");

        // Handle offset adjustments with gamepad 1
        if (robot.opMode.gamepad1.dpad_up) {
            yOffsetMeters += 0.01;
            robot.telemetry.addData("Y Offset Increased", String.format("%.3f m", yOffsetMeters));
            sleep(200); // Debounce
        }
        if (robot.opMode.gamepad1.dpad_down) {
            yOffsetMeters -= 0.01;
            robot.telemetry.addData("Y Offset Decreased", String.format("%.3f m", yOffsetMeters));
            sleep(200); // Debounce
        }
        if (robot.opMode.gamepad1.dpad_right) {
            xOffsetMeters += 0.01;
            robot.telemetry.addData("X Offset Increased", String.format("%.3f m", xOffsetMeters));
            sleep(200); // Debounce
        }
        if (robot.opMode.gamepad1.dpad_left) {
            xOffsetMeters -= 0.01;
            robot.telemetry.addData("X Offset Decreased", String.format("%.3f m", xOffsetMeters));
            sleep(200); // Debounce
        }
        if (robot.opMode.gamepad1.right_bumper) {
            headingOffsetDegrees += 1.0;
            robot.telemetry.addData("Heading Offset Increased", String.format("%.1f°", headingOffsetDegrees));
            sleep(200); // Debounce
        }
        if (robot.opMode.gamepad1.left_bumper) {
            headingOffsetDegrees -= 1.0;
            robot.telemetry.addData("Heading Offset Decreased", String.format("%.1f°", headingOffsetDegrees));
            sleep(200); // Debounce
        }

        // Movement test controls (needs to be handled by external code connected to buttons)
        if (robot.opMode.gamepad1.b) {
            readyForMovementTest = true;
            preMovePose = null;
            postMovePose = null;
            hasMovementTestCompleted = false;
            robot.telemetry.addData("Movement Test", "Ready to start");
            sleep(300); // Debounce
        }

        if (robot.opMode.gamepad1.x && readyForMovementTest && preMovePose != null && !hasMovementTestCompleted) {
            // Trigger test movement
            hasMovementTestCompleted = true;
            robot.telemetry.addData("Movement Test", "Moving...");

            // Execute a small forward movement (you need to integrate this with your button press)
            performTestMovement();

            sleep(300); // Debounce
        }

        if (robot.opMode.gamepad1.y && postMovePose != null) {
            // Reset the test
            readyForMovementTest = false;
            preMovePose = null;
            postMovePose = null;
            hasMovementTestCompleted = false;
            robot.telemetry.addData("Movement Test", "Reset");
            sleep(300); // Debounce
        }
    }

    // This method should be triggered when the user presses the X button during the test
    private void performTestMovement() {
        // Perform a simple forward movement to test localization
        // This could be integrated with a button press in your teleOp mode

        new SequentialCommandGroup(
                new InstantCommand(() -> robot.telemetry.addData("Test Movement", "Starting...")),
                // Move forward 10 inches
                new StrafeToPose(robot,
                        new Pose2d(
                                robot.mecanum.pose.position.x + 10,
                                robot.mecanum.pose.position.y,
                                robot.mecanum.pose.heading.toDouble()),
                        2.0), // 2 second timeout
                new InstantCommand(() -> robot.telemetry.addData("Test Movement", "Completed"))
        ).schedule();
    }

    private double normalizeAngleDegrees(double degrees) {
        double normalized = degrees;
        while (normalized > 180) normalized -= 360;
        while (normalized < -180) normalized += 360;
        return normalized;
    }

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
        robot.telemetry.addData("AprilTag Detect", "Ended");

        // Provide summary of calibration findings
        if (lastBotpose != null) {
            robot.telemetry.addLine("\n----- Final Calibration Values -----");
            robot.telemetry.addData("X Offset", String.format("%.3f m", xOffsetMeters));
            robot.telemetry.addData("Y Offset", String.format("%.3f m", yOffsetMeters));
            robot.telemetry.addData("Heading Offset", String.format("%.1f°", headingOffsetDegrees));
            robot.telemetry.addLine("\nAdd these values to your SensorPackage.updatePose() method");
        }
    }
}