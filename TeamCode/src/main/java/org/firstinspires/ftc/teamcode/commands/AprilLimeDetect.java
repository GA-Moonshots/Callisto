package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.util.Timing;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;

public class AprilLimeDetect extends CommandBase {
    private final Callisto robot;
    private final SensorPackage sensors;
    private Timing.Timer timer;

    private boolean foundPose = false;
    private Pose2d fromDetect = null;

    public AprilLimeDetect(Callisto robot) {
        this.robot = robot;
        this.sensors = robot.sensors;

        timer = new Timing.Timer((long) 3);

        addRequirements(sensors);
    }

    @Override
    public void initialize() {
        timer.start();
    }

    @Override
    public void execute() {
        robot.telemetry.addData("Detect Running:", true);
        LLResult result = sensors.getLLResult();
        if(result != null && result.isValid()) {
            double tx = result.getTx(); // How far the tag is from center horizontally
            double ty = result.getTy(); // Vertical offset
            double ta = result.getTa(); // How big the tag looks, usually correlating to distance

            robot.telemetry.addData("Target X", tx);
            robot.telemetry.addData("Target Y", ty);
            robot.telemetry.addData("Target Area", ta);

            robot.telemetry.addData("target id", result.getFiducialResults());



            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                robot.sensors.updatePose(botpose);


               foundPose = true;
            }
        } else {
            robot.telemetry.addData("limelight", "No Targets");

        }
    }

    @Override
    public void end(boolean interrupted){
        if(fromDetect != null){
            robot.telemetry.addData("Object is not null", true);
            robot.mecanum.pose = fromDetect;
            robot.telemetry.addData("Roadrunner(postion): ", robot.mecanum.pose.position.toString());
            robot.telemetry.addData("Roadrunner(heading): ", robot.mecanum.pose.heading.toString());
        }
        else {
            robot.telemetry.addData("Object is null:", true);
        }
    }

    @Override
    public boolean isFinished() {
        return foundPose && timer.done();
    }
}
