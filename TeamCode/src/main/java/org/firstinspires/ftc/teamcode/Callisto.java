package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.MoveToPose;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.util.AutoBotDriveyPants;
import org.firstinspires.ftc.teamcode.util.experiments.PureMecanum;


public class Callisto extends Robot {

    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;
    public boolean isRed;

    // SUBSYSTEMS
    public Mecanum mecanum;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;


    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     * @param opMode The selected operation mode
     */
    public Callisto(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);
        initTele();
    }

    // OVERLOADED CONSTRUCTOR THAT RESPONDS TO AUTONOMOUS OPMODE USER QUERY
    public Callisto(LinearOpMode opMode, boolean isRed) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.isRed = isRed;
        initAuto();
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {

        // MECANUM SETUP
            //TODO: Eventually we will change this to RoadRunner Mecanum?
        mecanum = new Mecanum(this, new Pose2d(new Vector2d(0,0),0));

        // Register subsystems
        // REGISTER THE SUBSYSTEM BEFORE THE DEFAULT COMMANDS
        register(mecanum);

        // Setting Default Commands
        mecanum.setDefaultCommand(new Drive(this));



        /*
                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/
        */
        Button aButtonP1 = new GamepadButton(player1, GamepadKeys.Button.A);
        aButtonP1.whenPressed(new InstantCommand(() -> {
            mecanum.toggleFieldCentric();
        }));

        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(new InstantCommand(() -> {
            mecanum.resetFieldCentricTarget();
        }));

        Button xButtonP1 = new GamepadButton(player1, GamepadKeys.Button.X);
        Button yButtonP1 = new GamepadButton(player1, GamepadKeys.Button.Y);
        Button dPadUpP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_UP);
        Button dPadDownP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_DOWN);
        Button dPadLeftP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);
        Button dPadRightP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

        /*
                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'
         */
    }

    public void initAuto(){
        Pose2d start;
        Pose2d ending = new Pose2d(new Vector2d(3,3), 0.0);

        // RED LEFT
        if(isRed)
            start = new Pose2d(new Vector2d(0,0), 0.0);
           // start = new Pose2d(new Vector2d(0, 0), 0.0);
        else
            start = new Pose2d(new Vector2d(0, 0), 0.0);


        mecanum = new Mecanum(this, start);


        register(mecanum);

        new SequentialCommandGroup(
                new MoveToPose(this,  ending)
        ).schedule();
    }
}
