package org.firstinspires.ftc.teamcode.util.wrappers;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.opencv.core.Point;
import org.openftc.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.apriltag.AprilTagDetectorJNI;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class Camera {
    // Instance variables
    public boolean isAprilTag = true;
    private List<AprilTagDetection> detections;
    private final AprilTagProcessor aprilTag;

    // Hardware
    private Callisto m_robot;
    private OpenCvCamera camera;

    public Camera (Callisto robot, Telemetry telemetry) {
        telemetry.addData("Camera Working", true);
        telemetry.update();

        // We instantiate the robot
        m_robot = robot;

        // April Tag
        aprilTag = new AprilTagProcessor.Builder()
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .build();

        // Camera Instantiation procedures
        int cameraMonitorViewId = m_robot.opMode.hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id",
                        m_robot.opMode.hardwareMap.appContext.getPackageName());

        WebcamName webcamName = m_robot.opMode.hardwareMap.get(WebcamName.class, Constants.WEBCAM_NAME);
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);;

        // FTC Dashboard Camera Set up stuff
        FtcDashboard.getInstance().startCameraStream(camera, 60);

        // Sets our camera pipeline
        camera.setPipeline(new ObjectDetectionPipeline(robot.telemetry, detections));

        // Opening the camera
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                // Usually this is where you'll want to start streaming from the camera (see section 4)
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    class ObjectDetectionPipeline extends OpenCvPipeline {
        private String detectedColor = "UNKNOWN"; // Variable to store the detected color
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

        public ObjectDetectionPipeline(Telemetry telemetry, List<AprilTagDetection> detections) {
            this.telemetry = telemetry;
            this.detections = (ArrayList<AprilTagDetection>) (detections);
            nativeAprilTagPtr = AprilTagDetectorJNI.createApriltagDetector(AprilTagDetectorJNI.TagFamily.TAG_36h11.string, 3, 3);
        }

        @Override
        public Mat processFrame(Mat input) {
            // Step 1: Process color detection
            Mat hsv = new Mat();
            Imgproc.cvtColor(input, hsv, Imgproc.COLOR_RGB2HSV);

            // Define the region of interest (center 25% of the frame)
            int centerX = input.cols() / 2;
            int centerY = input.rows() / 2;
            int regionWidth = input.cols() / 4;
            int regionHeight = input.rows() / 4;
            Rect reccyBoi = new Rect(centerX - regionWidth / 2, centerY - regionHeight / 2, regionWidth, regionHeight);
            Mat centerMat = hsv.submat(reccyBoi);

            // Determine the color in the center region
            if (isColorInRange(centerMat, new Scalar(20, 100, 150), new Scalar(35, 255, 255))) {
                detectedColor = "YELLOW";
            } else if (isColorInRange(centerMat, new Scalar(0, 150, 70), new Scalar(10, 255, 255)) ||
                    isColorInRange(centerMat, new Scalar(170, 150, 70), new Scalar(180, 255, 255))) {
                detectedColor = "RED";
            } else if (isColorInRange(centerMat, new Scalar(100, 150, 70), new Scalar(140, 255, 255))) {
                detectedColor = "BLUE";
            } else {
                detectedColor = "UNKNOWN";
            }

            // Draw rectangles around the detected colors for visualization
            Scalar drawColor = new Scalar(255, 0, 0); // Default to blue
            switch (detectedColor) {
                case "RED":
                    drawColor = new Scalar(0, 0, 255);
                    Imgproc.rectangle(input, reccyBoi, drawColor, 2);
                    break;
                case "BLUE":
                    drawColor = new Scalar(0, 0, 255);
                    Imgproc.rectangle(input, reccyBoi, drawColor, 2);
                    break;
                case "YELLOW":
                    drawColor = new Scalar(0, 0, 255);
                    Imgproc.rectangle(input, reccyBoi, drawColor, 2);
                    break;
            }

            // Step 2: Apritag processing
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

            for (org.openftc.apriltag.AprilTagDetection detection : detections) {
                draw2dSquare(input, detection.corners);
            }

            // Step 3: Return the modified frame for streaming
            return input;
        }

        // Method to get the detected color
        public String getDetectedColor() {
            return detectedColor;
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