// AlignToGamePiece.java

package org.firstinspires.ftc.teamcode.commands;

import static com.arcrobotics.ftclib.util.Timing.*;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.opencv.core.Point;

public class AlignToGamePiece extends CommandBase {

    private final Callisto robot;
    private final Mecanum mecanum;
    private final SensorPackage sensors;
    private final boolean isRed;

    // Adjustable Parameters (Magic Numbers)
    private final double Kp;
    private final double minPower;
    private final double tolerance;
    private final int clawOffsetX;
    private final int clawOffsetY;

    // =========================================================================
    // *** ADJUST THESE OFFSETS TO MATCH THE CLAW'S GRAB LOCATION ***
    // The horizontal offset (in pixels) from the camera center to the claw's grab point
    // The vertical offset (in pixels) from the camera center to the claw's grab point
    // =========================================================================

    private final double targetX;
    private final double targetY;

    private boolean finished = false;
    private final Timer timer;

    public AlignToGamePiece(Callisto robot) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.sensors = robot.sensors;
        this.isRed = robot.isRed;

        // TODO: determine constants
        this.Kp = 1;
        this.minPower = 1;
        this.tolerance = 1;
        this.clawOffsetX = 10;
        this.clawOffsetY = 10;
        double frameWidth = 400;
        double frameHeight = 400;

        // =========================================================================
        // *** ADJUST THESE OFFSETS TO MATCH THE CLAW'S GRAB LOCATION ***
        // The horizontal offset (in pixels) from the camera center to the claw's grab point
        // The vertical offset (in pixels) from the camera center to the claw's grab point
        // =========================================================================

        this.targetX = frameWidth / 2.0 + clawOffsetX;
        this.targetY = frameHeight / 2.0 + clawOffsetY;

        // Initialize the timer with default timeout
        this.timer = new Timer((long) Constants.DEFAULT_TIMEOUT);

        addRequirements(mecanum);
    }

    @Override
    public void initialize() {
        // Start the timer
        timer.start();
    }

    @Override
    public void execute() {
        // TODO: Verify color before honing in on centroid
        // TODO: Smooth out the sweeping pattern to find the right color

        // Get the detected centroid
        Point detectedCentroid = sensors.camera.getDetectedCentroid();
        String detectedColor = sensors.camera.getDetectedColor();

        // If no object detected, stop the robot
        if (detectedCentroid.x == -1 && detectedCentroid.y == -1) {
            mecanum.stop();
            return;
        }

        // Calculate errors
        double errorX = targetX - detectedCentroid.x;
        double errorY = targetY - detectedCentroid.y;

        // Calculate control outputs
        double powerX = Kp * errorX;
        double powerY = Kp * errorY;

        // Apply minimum power thresholds
        if (Math.abs(powerX) < minPower) {
            powerX = 0;
        }
        if (Math.abs(powerY) < minPower) {
            powerY = 0;
        }

        // Command the drivetrain
        mecanum.drive(powerY, powerX, 0); // Adjust parameters based on your drive method

        // Check if aligned within tolerance
        if (Math.abs(errorX) < tolerance && Math.abs(errorY) < tolerance) {
            mecanum.stop();
            finished = true;
        }
    }

    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    @Override
    public void end(boolean interrupted) {
        mecanum.stop();
    }
}
