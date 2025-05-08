package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.Pose2d;
import com.seattlesolvers.solverslib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.DriveyMcDriverson;
import org.firstinspires.ftc.teamcode.util.PersistentPoseManager;

public class SavePoseCommand extends CommandBase {
    private final Mecanum driveSubsystem;

    public SavePoseCommand(Callisto robot) {
        this.driveSubsystem = robot.mecanum;
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean interrupted) {
        Pose2d finalPose = driveSubsystem.pose;
        PersistentPoseManager.savePose(finalPose);
    }
}
