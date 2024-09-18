package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.experiments.PureMecanum;
import org.firstinspires.ftc.teamcode.util.Constants;

public class Drive extends CommandBase {
    private final Mecanum mecanum;
    private GamepadEx player1;
    private Callisto robot;

    private double strafeSpeed;
    private double forwardSpeed;
    private double turnSpeed;

    public Drive(Callisto robot) {
        // pulling off some handy references
        this.robot = robot;
        this.mecanum = robot.mecanum;
        player1 = robot.player1;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robot.mecanum);
    }

    @Override
    public void initialize(){
        // Do you need to set up anything when the command gets first added to the scheduler?
    }

    @Override
    public void execute() {

        // you can go digging for the opMode controller
        double speedMod = robot.opMode.gamepad1.right_bumper ? 0.2 : 1; // slow mode

        double forward = applyDeadZone(player1.getLeftY());
        double strafe = applyDeadZone(player1.getLeftX());
        double turn = applyDeadZone(player1.getRightX());

        // Drive the robot with adjusted inputs:
        mecanum.drive(forward * speedMod, strafe * speedMod, turn * speedMod);
    }


    public boolean isFinished() {
        // currently just a placeholder
        return false;
    }

    /**
     * Helper function for applying dead zone
     */
    private double applyDeadZone(double input) {
        return Math.abs(input) <= Constants.INPUT_THRESHOLD ? 0.0d : input;
    }
}
