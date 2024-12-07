package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.Constants;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.util.Timing;

public class Rotate extends CommandBase {

    // REFERENCES
    private final Callisto robot;
    private final Mecanum mecanum;

    // ASSETS
    private final double targetAngle; // The target heading angle in radians
    protected Action action;
    protected boolean finished = false;

    // TIMER
    protected Timing.Timer timer;

    // FTC DASHBOARD
    private final FtcDashboard dashboard;

    // Constructor to initialize the command
    public Rotate(Callisto robot, double targetAngle) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.targetAngle = targetAngle;

        // default timeout
        timer = new Timing.Timer((long) Constants.DEFAULT_TIMEOUT);

        dashboard = FtcDashboard.getInstance();
        addRequirements(mecanum);
    }

    public Rotate(Callisto robot, double targetAngle, double timeout) {
        this.robot = robot;
        this.mecanum = robot.mecanum;
        this.targetAngle = targetAngle;

        timer = new Timing.Timer((long) timeout);

        dashboard = FtcDashboard.getInstance();
        addRequirements(mecanum);
    }

    // Initialize method to build the rotate action
    @Override
    public void initialize() {
        // Start the timer when the command begins
        timer.start();

        // Build the rotation action from the current pose to the target angle
        action = mecanum.actionBuilder(mecanum.pose)
                .turnTo(Math.toRadians(targetAngle))
                .build();
    }

    // The execute method updates the rotation following
    @Override
    public void execute() {
        // Create a telemetry packet for FTC Dashboard
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Current Heading", mecanum.pose.heading.toDouble());
        packet.put("Target Heading", targetAngle);

        // Display a marker for the current heading
        packet.fieldOverlay()
                .setStrokeWidth(1)
                .setStroke("Red")
                .fillCircle(mecanum.pose.position.x, mecanum.pose.position.y, 3);

        // Run the action and update if finished
        finished = !action.run(packet);
        dashboard.sendTelemetryPacket(packet);
    }

    // Check if the command has finished
    @Override
    public boolean isFinished() {
        return finished || timer.done();
    }

    // Stop the robot once the command ends
    @Override
    public void end(boolean interrupted) {
        mecanum.stop();
    }
}
