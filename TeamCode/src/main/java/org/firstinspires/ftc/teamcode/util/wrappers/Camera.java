package org.firstinspires.ftc.teamcode.util.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class Camera implements AutoCloseable {
    private final Callisto m_robot;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;
    private long lastDetectionTime = 0;

    public Camera(Callisto robot, Telemetry telemetry) {
        m_robot = robot;
        telemetry.addData("Camera Status", "Initializing...");
        telemetry.update();

        // Create the AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Create the vision portal with live view
        WebcamName webcamName = m_robot.opMode.hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME);
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcamName)
                .addProcessor(aprilTagProcessor)
                .build();

        telemetry.addData("Camera Status", "Initialized");
        telemetry.update();
    }

    /**
     * Gets the latest AprilTag detections if they are fresh (within the last second)
     * @return ArrayList of AprilTagDetection or null if detections are stale or nonexistent
     */
    public ArrayList<AprilTagDetection> getLatestFreshDetections() {
        List<AprilTagDetection> currentDetections = aprilTagProcessor.getDetections();
        long currentTime = System.currentTimeMillis();

        if (currentDetections != null && !currentDetections.isEmpty()) {
            lastDetectionTime = currentTime;
            return new ArrayList<>(currentDetections);
        }

        // Return null if detections are stale (older than 1 second) or nonexistent
        if (currentTime - lastDetectionTime > 1000) {
            return null;
        }

        return new ArrayList<>(currentDetections);
    }

    @Override
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}