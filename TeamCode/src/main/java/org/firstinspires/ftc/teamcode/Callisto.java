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
import org.firstinspires.ftc.teamcode.commands.DumpThenLevel;
import org.firstinspires.ftc.teamcode.commands.ExtendArm;
import org.firstinspires.ftc.teamcode.commands.ExtendShoulderByAmount;
import org.firstinspires.ftc.teamcode.commands.RaiseLift;
import org.firstinspires.ftc.teamcode.commands.RaiseShoulder;
import org.firstinspires.ftc.teamcode.commands.MoveToPose;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;
import org.firstinspires.ftc.teamcode.util.experiments.ServoTest;

import static org.firstinspires.ftc.teamcode.subsystems.Mecanum.botType;


public class Callisto extends Robot {

    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;
    public boolean isRed;

    // SUBSYSTEMS
    public Mecanum mecanum;
    public SensorPackage sensors;
    public Arm arm;
    public Lift lift;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public ServoTest servo;

    public Blinkin blinkin;


    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     * @param opMode The selected operation mode
     */
    public Callisto(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);
        initTele();
    }

    // OVERLOADED CONSTRUCTOR THAT RESPONDS TO AUTONOMOUS OPMODE USER QUERY
    public Callisto(LinearOpMode opMode, boolean isRed) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.isRed = isRed;
        initAuto();
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {
        // TODO: restore sensors
        sensors = new SensorPackage(this);
        mecanum = new Mecanum(this, new Pose2d(new Vector2d(0,0),0));

        //blinkin = new Blinkin(this);
        arm = new Arm(this);
        lift = new Lift(this);

        // Register subsystems
        // REGISTER THE SUBSYSTEM BEFORE THE DEFAULT COMMANDS
        // TODO: add sensors back in
        register(mecanum, arm, lift, sensors);

        // Setting Default Commands
        mecanum.setDefaultCommand(new Drive(this));
        arm.setDefaultCommand(new RaiseShoulder(this));
        lift.setDefaultCommand(new RaiseLift(this, 0));

        // If botType = true then it is small bot
        // if botType = false then it is large bot
        botType = false;


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
        (_)                `\___/'-50  -50      */

        Button aButtonP2 = new GamepadButton(player2, GamepadKeys.Button.A);
        aButtonP2.whenPressed(new InstantCommand(() -> {
            arm.toggleClaw();
        }));

        Button bButtonP2 = new GamepadButton(player2, GamepadKeys.Button.B);
        bButtonP2.whenPressed(new InstantCommand(() -> {
            lift.dumpBasket();
        }));

        Button yButtonP2 = new GamepadButton(player2, GamepadKeys.Button.Y);
        yButtonP2.whenPressed(new InstantCommand(() -> {
            lift.levelBasket();
        }));

        Button rightBumperP2 = new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER);
        rightBumperP2.whenPressed(new SequentialCommandGroup(
//                new ExtendShoulderByAmount(this, 0.6),
//                new ExtendShoulderByAmount(this, -0.5),
                new RaiseLift(this, 500),
                new DumpThenLevel(this, 5),
                new RaiseLift(this, 0),
                new RaiseLift(this, 0)
        ));

        Button leftBumperP2 = new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);
        leftBumperP2.whenHeld(new ExtendArm(this));

        Button downDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);
        downDpadP2.whenPressed(new RaiseLift(this, 0));

        Button leftDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT);
        leftDpadP2.whenPressed(new RaiseLift(this, 250));

        Button upDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        upDpadP2.whenPressed(new RaiseLift(this, 500));

    }

    public void initAuto(){
        //sensors = new SensorPackage(this);

        Pose2d start;
        Pose2d ending = new Pose2d(new Vector2d(0,0), 180);
        Pose2d next = new Pose2d(new Vector2d(15,0), 180);

        // RED LEFT

        start = new Pose2d(new Vector2d(0,0), 0.0);

        mecanum = new Mecanum(this, start);

        // TODO: Add sensors back in
        register(mecanum);

        new SequentialCommandGroup(
                new MoveToPose(this, ending)
        ).schedule();
    }
}
