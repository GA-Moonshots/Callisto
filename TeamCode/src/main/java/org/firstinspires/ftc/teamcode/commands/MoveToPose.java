package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.RRMecanum;
import org.firstinspires.ftc.teamcode.util.HardwareNames;

import com.acmerobotics.roadrunner.Trajectory;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveToPose extends CommandBase {

    // REFERENCES
    private Callisto robot;
    //TODO: Ask Mr. A is the naming convention of this a problem
    private RRMecanum drive;
    // ASSETS
    private final Pose2d targetPose; // The target position and heading
    protected Action action;
    protected boolean finished = false;
    // TIMER
    protected final double timeout;
    protected Timing.Timer timer;


    // Constructor to initialize the command
    public MoveToPose(Callisto robot, Pose2d targetPose) {
        this.robot = robot;
        this.drive = robot.roadRunner;
        this.targetPose = targetPose;

        // default timeout
        this.timeout = HardwareNames.DEFAULT_TIMEOUT;

        addRequirements(drive);
    }

    public MoveToPose(Callisto robot, Pose2d targetPose, double timeout) {
        this.robot = robot;
        this.drive = robot.roadRunner;
        this.targetPose = targetPose;

        this.timeout = timeout;

        addRequirements(drive);

    }

    // Initialize method to build the trajectory
    @Override
    public void initialize() {
        // only start the timer once the command is run and not built
        timer.start();

        // Build the trajectory from the current pose to the target pose
        action = drive.actionBuilder(drive.pose)
                .splineTo(targetPose.position, targetPose.heading)
                .build();
    }

    // The execute method keeps updating the trajectory following
    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("x", drive.pose.position.x);
        packet.put("y", drive.pose.position.y);
        packet.put("heading", drive.pose.heading.toDouble());

        // Use the telemetryPacket with the action's run method:
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return finished || timer.elapsedTime() > timeout;
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        drive.stop();
    }
}