package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;

public class SensorPackage extends SubsystemBase {
    // INSTANCE VARIABLES
    private Callisto robot;
    private final Telemetry telemetry;
    public Camera camera;

    public SensorPackage(Callisto robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;

        try {
            this.camera = new Camera(robot, robot.opMode.telemetry);
        } catch (Exception ignored) {}

    }

    @Override
    public void periodic() {
        // THIS IS THE ONLY TELEMETRY.UPDATE IN OUR CODEBASE
        telemetry.update();
    }
}
