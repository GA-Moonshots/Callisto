package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Robot;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;

public class AprilLocalization extends CommandBase {
    private Callisto robot;
    private Camera camera;

    public AprilLocalization(Callisto robot) {
        this.robot = robot;
        camera = this.robot.sensors.camera;

        addRequirements(this.robot.sensors);
    }

    @Override
    public void execute() {
        if (camera.getLatestFreshDetections() != null && !camera.getLatestFreshDetections().isEmpty()) {
            robot.telemetry.addData("April Tag Location", camera.getLatestFreshDetections().get(0).ftcPose);
        }
    }
}
