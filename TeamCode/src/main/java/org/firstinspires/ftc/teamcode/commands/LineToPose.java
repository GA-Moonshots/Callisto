package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.Constants;

import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

public class LineToPose extends CommandBase {

    // REFERENCES
    private Callisto robot;
    private Mecanum mecanum;

    // ASSETS
    private final Pose2d targetPose; // The target position and heading
    protected Action action;
    protected boolean finished = false;

    // TIMER
    protected Timing.Timer timer;

    // FTC DASHBOARD
    private FtcDashboard dashboard;

    // Constructor to initialize the command
    public LineToPose(Callisto robot, Pose2d targetPose) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.targetPose = targetPose;

        // default timeout
        timer = new Timing.Timer((long)Constants.DEFAULT_TIMEOUT);

        addRequirements(mecanum);
    }

    public LineToPose(Callisto robot, Pose2d targetPose, double timeout) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.targetPose = targetPose;
        this.dashboard = FtcDashboard.getInstance();

        // Set the timeout for the command
        timer = new Timing.Timer((long)timeout);

        // this will allow us to interrupt any other command that may be running
        addRequirements(mecanum);

    }

    // Initialize method to build the trajectory
    @Override
    public void initialize() {
        // only start the timer once the command is run and not built
        timer.start();

        // Build the trajectory from the current pose to the target pose
        action = mecanum.actionBuilder(mecanum.pose).lineToX(targetPose.position.x,
                mecanum.defaultVelConstraint).build();

    }

    // The execute method keeps updating the trajectory following
    @Override
    public void execute() {
        // Create Packet for dashboard
        TelemetryPacket packet = new TelemetryPacket();


        // Log the target and current position
        packet.put("Target X", targetPose.position.x);
        packet.put("Target Y", targetPose.position.y);
        packet.put("Current X", mecanum.pose.position.x);
        packet.put("Current Y", mecanum.pose.position.y);
        packet.put("Current Heading", mecanum.pose.heading.toDouble());


        packet.put("x", mecanum.pose.position.x);
        packet.put("y", mecanum.pose.position.y);
        packet.put("heading", mecanum.pose.heading.toDouble());

        packet.fieldOverlay()
                .setStrokeWidth(1).setStroke("Blue")
                .fillCircle(mecanum.pose.position.x, mecanum.pose.position.y,3);

        // Use the telemetryPacket with the action's run method:
        finished = !action.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        // Update the action based on the current pose for the next cycle
        action = mecanum.actionBuilder(mecanum.pose).lineToX(targetPose.position.x,
                mecanum.defaultVelConstraint).build();
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        // Stop the drive if interrupted or completed
        mecanum.stop();
    }
}