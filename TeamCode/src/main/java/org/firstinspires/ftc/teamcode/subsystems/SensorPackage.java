package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.wrappers.Camera;

public class SensorPackage extends SubsystemBase {
    // INSTANCE VARIABLES
    private Callisto robot;
    private final Telemetry telemetry;
    private Camera camera;
    public DistanceSensor rearDistance;
    public DistanceSensor leftDistance;
    public DistanceSensor rightDistance;

    public SensorPackage(Callisto robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
        //this.camera = new Camera(robot, telemetry);
        try {
            this.camera = new Camera(robot, robot.opMode.telemetry);
        } catch (Exception ignored) {}



        // instantiate distance sensors using our wrapper
//        this.rearDistance = new DistanceSensor(robot.opMode, Constants.REAR_DIST_NAME);
//        this.rightDistance = new DistanceSensor(robot.opMode, Constants.RIGHT_DIST_NAME);
//        this.leftDistance = new DistanceSensor(robot.opMode, Constants.LEFT_DIST_NAME);
    }


    @Override
    public void periodic() {
        robot.mecanum.postEncoderData();
        telemetry.update();
    }
}
