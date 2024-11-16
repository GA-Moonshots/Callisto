package org.firstinspires.ftc.teamcode.util.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.opencv.core.MatOfPoint;
import org.opencv.imgproc.Moments; // Correct import for Moments
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Camera {
    // Instance variables
    public boolean isAprilTag = true;
    private List<AprilTagDetection> detections;
    private final AprilTagProcessor aprilTag;

    // Hardware
    private Callisto m_robot;
    private OpenCvCamera camera;
    private ObjectDetectionPipeline pipeline;

    public Camera(Callisto robot, Telemetry telemetry) {
        telemetry.addData("Camera Working", true);
        telemetry.update();

        // We instantiate the robot
        m_robot = robot;

        // April Tag
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Camera Instantiation procedures
        int cameraMonitorViewId = m_robot.opMode.hardwareMap.appContext.getResources()
                .getIdentifier("cameraMonitorViewId", "id",
                        m_robot.opMode.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = m_robot.opMode.hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

        // FTC Dashboard Camera Set up stuff
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        // Sets our camera pipeline
        pipeline = new ObjectDetectionPipeline(robot.telemetry);
        camera.setPipeline(pipeline);

        // Opening the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                // Start streaming from the camera
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    /**
     * Returns the centroid point of the detected game piece.
     *
     * A centroid is the arithmetic mean position of all the points in a shape. In image processing,
     * the centroid of a contour (detected object) is calculated based on the spatial moments of the contour.
     * It represents the center point of the object.
     *
     * @return The centroid point. If no object is detected, returns (-1, -1).
     */
    public Point getDetectedCentroid() {
        return pipeline.getDetectedCentroid();
    }

    /**
     * Returns the color of the detected game piece.
     *
     * @return The detected color as a String ("RED", "BLUE", "YELLOW", or "UNKNOWN").
     */
    public String getDetectedColor() {
        return pipeline.getDetectedColor();
    }

    class ObjectDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "UNKNOWN"; // Variable to store the detected color
        private Point detectedCentroid = new Point(-1, -1); // Centroid of the detected object
        private long nativeAprilTagPtr;
        private final Telemetry telemetry;

        private ArrayList<AprilTagDetection> detections = new ArrayList<>();
        private Mat grey = new Mat();

        private ArrayList<AprilTagDetection> detectionsUpdate = new ArrayList<>();
        private final Object detectionsUpdateSync = new Object();

        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;

        double tagSize = 0.166;

        private float decimation;
        private boolean needToSetDecimation;
        private final Object decimationSync = new Object();

        public ObjectDetectionPipeline(Telemetry telemetry) {
            this.telemetry = telemetry;
            nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(
                    AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Convert the image from RGB to HSV color space
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define color ranges
            Scalar lowerYellow = new Scalar(20, 100, 100);
            Scalar upperYellow = new Scalar(30, 255, 255);

            Scalar lowerRed1 = new Scalar(0, 150, 70);
            Scalar upperRed1 = new Scalar(10, 255, 255);
            Scalar lowerRed2 = new Scalar(170, 150, 70);
            Scalar upperRed2 = new Scalar(180, 255, 255);

            Scalar lowerBlue = new Scalar(100, 150, 70);
            Scalar upperBlue = new Scalar(140, 255, 255);

            // Create masks for each color
            Mat maskYellow = new Mat();
            Mat maskRed1 = new Mat();
            Mat maskRed2 = new Mat();
            Mat maskRed = new Mat();
            Mat maskBlue = new Mat();
            Mat mask = new Mat();

            Core.inRange(hsv, lowerYellow, upperYellow, maskYellow);
            Core.inRange(hsv, lowerRed1, upperRed1, maskRed1);
            Core.inRange(hsv, lowerRed2, upperRed2, maskRed2);
            Core.inRange(hsv, lowerBlue, upperBlue, maskBlue);

            // Combine red masks
            Core.bitwise_or(maskRed1, maskRed2, maskRed);

            // Combine masks based on team color
            if (m_robot.isRed) {
                Core.bitwise_or(maskRed, maskYellow, mask);
            } else {
                Core.bitwise_or(maskBlue, maskYellow, mask);
            }

            // Find contours in the mask
            List<MatOfPoint> contours = new ArrayList<>();
            Mat hierarchy = new Mat();
            Imgproc.findContours(mask, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

            if (!contours.isEmpty()) {
                // Find the largest contour
                double maxArea = 0;
                MatOfPoint largestContour = null;
                for (MatOfPoint contour : contours) {
                    double area = Imgproc.contourArea(contour);
                    if (area > maxArea) {
                        maxArea = area;
                        largestContour = contour;
                    }
                }

                if (largestContour != null) {
                    // Calculate the centroid of the largest contour
                    Moments moments = Imgproc.moments(largestContour);
                    int cX = (int) (moments.m10 / moments.m00);
                    int cY = (int) (moments.m01 / moments.m00);
                    detectedCentroid = new Point(cX, cY);

                    // Draw the largest contour and centroid
                    Imgproc.drawContours(input, Arrays.asList(largestContour), -1, new Scalar(0, 255, 0), 2);
                    Imgproc.circle(input, detectedCentroid, 5, new Scalar(255, 0, 0), -1);

                    // Determine the detected color
                    if (Core.countNonZero(maskYellow) > 0) {
                        detectedColor = "YELLOW";
                    } else if (Core.countNonZero(maskRed) > 0) {
                        detectedColor = "RED";
                    } else if (Core.countNonZero(maskBlue) > 0) {
                        detectedColor = "BLUE";
                    } else {
                        detectedColor = "UNKNOWN";
                    }
                }
            } else {
                detectedCentroid = new Point(-1, -1);
                detectedColor = "UNKNOWN";
            }

            // AprilTag processing (if needed)
            // Convert to greyscale
            Imgproc.cvtColor(input, grey, Imgproc.COLOR_RGBA2GRAY);

            synchronized (decimationSync) {
                if (needToSetDecimation) {
                    AprilTagDetectorJNI.setApriltagDetectorDecimation(nativeAprilTagPtr, decimation);
                    needToSetDecimation = false;
                }
            }

            // Run AprilTag
            detections = AprilTagDetectorJNI.runAprilTagDetectorSimple(nativeAprilTagPtr, grey, tagSize, fx, fy, cx, cy);
            synchronized (detectionsUpdateSync) {
                detectionsUpdate = detections;
            }

            for (AprilTagDetection detection : detections) {
                draw2dSquare(input, detection.corners);
            }

            return input;
        }

        /**
         * Returns the color of the detected game piece.
         *
         * @return The detected color as a String ("RED", "BLUE", "YELLOW", or "UNKNOWN").
         */
        public String getDetectedColor() {
            return detectedColor;
        }

        /**
         * Returns the centroid point of the detected game piece.
         *
         * A centroid is the arithmetic mean position of all the points in a shape. In image processing,
         * the centroid of a contour (detected object) is calculated based on the spatial moments of the contour.
         * It represents the center point of the object.
         *
         * @return The centroid point. If no object is detected, returns (-1, -1).
         */
        public Point getDetectedCentroid() {
            return detectedCentroid;
        }

        // Helper method to check if a color is within a specified range
        private boolean isColorInRange(Mat mat, Scalar lowerBound, Scalar upperBound) {
            Mat mask = new Mat();
            Core.inRange(mat, lowerBound, upperBound, mask);
            double nonZeroCount = Core.countNonZero(mask);
            double totalPixels = mat.total();
            return (nonZeroCount / totalPixels) > 0.5; // Threshold to determine if the color is present
        }

        @Override
        protected void finalize() {
            // Might be null if createApriltagDetector() threw an exception
            if (nativeAprilTagPtr != 0) {
                // Delete the native context we created in the constructor
                AprilTagDetectorJNI.releaseApriltagDetector(nativeAprilTagPtr);
                nativeAprilTagPtr = 0;
            } else {
                System.out.println("AprilTagDetectionPipeline.finalize(): nativeAprilTagPtr was NULL");
            }
        }

        public ArrayList<AprilTagDetection> getLatestDetections() {
            return detections;
        }

        public ArrayList<AprilTagDetection> getDetectionsUpdate() {
            synchronized (detectionsUpdateSync) {
                ArrayList<AprilTagDetection> ret = detectionsUpdate;
                detectionsUpdate = null;
                return ret;
            }
        }

        void draw2dSquare(Mat buf, Point[] points) {
            Scalar blue = new Scalar(7, 197, 235, 255);
            Imgproc.rectangle(buf, points[0], points[2], blue, 3);
        }
    }
}
