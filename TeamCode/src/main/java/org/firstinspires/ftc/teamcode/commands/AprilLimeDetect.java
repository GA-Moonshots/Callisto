package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Limelight;

import java.util.List;

public class AprilLimeDetect extends CommandBase {
    private final Callisto robot;
    private final Limelight limelight;

    public AprilLimeDetect(Callisto robot) {
        this.robot = robot;
        this.limelight = robot.limelight;

        addRequirements(limelight);
    }

    @Override
    public void execute() {
        LLResult result = limelight.limelightRun();
        if(result != null && result.isValid()) {
            double tx = result.getTx(); // How far the tag is from center horizontally
            double ty = result.getTy(); // Vertical offset
            double ta = result.getTa(); // How big the tag looks, usually correlating to distance

            robot.telemetry.addData("Target X", tx);
            robot.telemetry.addData("Target Y", ty);
            robot.telemetry.addData("Target Area", ta);

            robot.telemetry.addData("target id", result.getFiducialResults());

//            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
//            for (LLResultTypes.FiducialResult fiducial : fiducials) {
//                int id = fiducial.getFiducialId();
//                robot.telemetry.addData("loop staus", true);
//                robot.telemetry.addData("Fiducial ID", id);
//                fiducial.getRobotPoseFieldSpace()
//            }

            if(result != null && result.isValid()) {
                Pose3D botpose = result.getBotpose();
                if (botpose != null) {
                    double x = botpose.getPosition().x;
                    double y = botpose.getPosition().y;
                    double theta = botpose.getOrientation().getYaw(AngleUnit.DEGREES);
                    robot.telemetry.addData("MT1 Location", "(" + (x * 39.3701) + "x inches, " + (y * 39.3701) + "y inches)");
                }
            }
        } else {
            robot.telemetry.addData("limelight", "No Targets");
        }
    }
}
