package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.Robot;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.button.Button;
import com.seattlesolvers.solverslib.command.button.GamepadButton;
import com.seattlesolvers.solverslib.command.button.Trigger;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.SplineToPose;
import org.firstinspires.ftc.teamcode.util.experiments.AprilLimeDetect;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.StrafeToPose;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByPlayer;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderDown;
import org.firstinspires.ftc.teamcode.commands.intake.TransferBlock;
import org.firstinspires.ftc.teamcode.commands.RotateByIMU;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.commands.lift.LiftLowerRTP;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.commands.lift.LiftRaiseRTP;
import org.firstinspires.ftc.teamcode.util.experiments.ServoTest;

public class Callisto extends Robot {

    // INSTANCE VARIABLES
    public LinearOpMode opMode;
    public GamepadEx player1;
    public GamepadEx player2;
    public boolean isRed;
    public boolean left;

    // SUBSYSTEMS
    public Mecanum mecanum;
    public SensorPackage sensors;
    public Lift lift;
    public Intake intake;

    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    public ServoTest servo;
    public Pose2d startOfTele;

    /**
     * Welcome to the Command pattern. Here we assemble the robot and kick-off the command
     *
     * @param opMode The selected operation mode
     */
    public Callisto(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        player1 = new GamepadEx(opMode.gamepad1);
        player2 = new GamepadEx(opMode.gamepad2);
        // TODO : load pose from persistent pose manager
        initTele();
    }

    // OVERLOADED CONSTRUCTOR THAT RESPONDS TO AUTONOMOUS OPMODE USER QUERY
    public Callisto(LinearOpMode opMode, boolean isRed, boolean left) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.isRed = isRed;
        this.left = left;
        initAuto();
    }

    /**
     * Set teleOp's default commands and player control bindings
     */
    public void initTele() {
        // JUST FOR TESTING
        startOfTele = new Pose2d(new Vector2d(-36, -62), Math.toRadians(180));
        mecanum = new Mecanum(this, startOfTele);
        lift = new Lift(this);
        intake = new Intake(this);
        sensors = new SensorPackage(this);
        sensors.enableAprilTagTracking();

        // Register subsystems
        // REGISTER THE SUBSYSTEM BEFORE THE DEFAULT COMMANDS
        register(mecanum, lift, intake, sensors);

        // Setting Default Commands. When not doing anything, respond to the controller
        mecanum.setDefaultCommand(new Drive(this));
        intake.setDefaultCommand(new IntakeShoulderByPlayer(this));

        /*

                .__                                      ____
        ______  |  |  _____   ___.__.  ____ _______     /_   |
        \____ \ |  |  \__  \ <   |  |_/ __ \\_  __ \     |   |
        |  |_> >|  |__ / __ \_\___  |\  ___/ |  | \/     |   |
        |   __/ |____/(____  // ____| \___  >|__|        |___|
        |__|               \/ \/          \/

        */

        // BUTTON A -- TOGGLE FIELD / ROBOT CENTRIC
        Button aButtonP1 = new GamepadButton(player1, GamepadKeys.Button.A);
        aButtonP1.whenPressed(new InstantCommand(() -> {
            mecanum.toggleFieldCentric();
        }));

        // BUTTON B
        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(
                new StrafeToPose(this, new Pose2d(-50, 0, 180))
        );

        // BUTTON X
        Button xButtonP1 = new GamepadButton(player1, GamepadKeys.Button.X);
        xButtonP1.whenPressed(
                new SplineToPose(this, new Pose2d(50, 0, 180))
        );

        // BUTTON Y -- Reset to Field centric
        Button yButtonP1 = new GamepadButton(player1, GamepadKeys.Button.Y);
        yButtonP1.whenPressed(
                new InstantCommand(() -> {
                    mecanum.resetFieldCentricTarget();
                })
        );

        Button dPadUpP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_UP);
        // !!! - APRIL TAG TESTING --- !!!!
        dPadUpP1.whenPressed(new AprilLimeDetect(this));

//        Button dPadDownP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_DOWN);
//        Button dPadLeftP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);
//        Button dPadRightP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

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

        // BUTTON B -- DUMP BASKET
        Button bButtonP2 = new GamepadButton(player2, GamepadKeys.Button.B);
        bButtonP2.whenPressed(new InstantCommand(() -> {
            lift.dumpBasket();
        }));

        // BUTTON Y -- LEVELS BASKET
        Button yButtonP2 = new GamepadButton(player2, GamepadKeys.Button.Y);
        yButtonP2.whenPressed(new InstantCommand(() -> {
            lift.levelBasket();
        }));

        //  LEFT BUMPER -- transfer block to the basket
        Button leftBumperP2 = new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);
        leftBumperP2.whenPressed(new TransferBlock(this));

        // LEFT TRIGGER -- SHOULDER DOWN
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whenActive(new IntakeShoulderDown(this));

        // DPAD DOWN -- LIFT LOWER
        Button downDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);
        downDpadP2.whenPressed(new LiftLowerRTP(this));


        // DPAD UP -- LIFT RAISE
        Button upDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        upDpadP2.whenPressed(new SequentialCommandGroup(
                // we always lower first to reset encoder
                new LiftLowerRTP(this),
                new LiftRaiseRTP(this, Constants.HIGH_HEIGHT)
        ));

        // RIGHT BUMPER -- NEGATIVE SPIN INTAKE
        Button rightBumperP2 = new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER);
        rightBumperP2.whenHeld(new InstantCommand(() -> {
            intake.setSpinSpeed(Constants.INTAKE_SPIN_SPEED_BACK);
        }));
        // RIGHT BUMPER RELEASE -- SPIN STOP
        rightBumperP2.whenReleased(new InstantCommand(() -> {
            // these servo functions don't know when to stop, so we kill 'em on release
            intake.setSpinSpeed(0);
        }));

        // RIGHT TRIGGER -- SPIN INTAKE
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightTriggerP2.whenActive(new InstantCommand(() -> {
            intake.setSpinSpeed(Constants.INTAKE_SPIN_SPEED_FORWARD);
        }));

        // RIGHT TRIGGER RELEASE -- SPIN STOP
        rightTriggerP2.whenInactive(new InstantCommand(() -> {
            // these servo functions don't know when to stop, so we kill 'em on release
            intake.setSpinSpeed(0.0);
        }));

        // JOYSTICK LEFT - nothing
        Button joyStickLeftTrigger = new GamepadButton(player2, GamepadKeys.Button.LEFT_STICK_BUTTON);

        // JOYSTICK RIGHT - nothing
        Button joyStickRightTrigger = new GamepadButton(player2, GamepadKeys.Button.RIGHT_STICK_BUTTON);
    }

    /**
     * -----------------
     * ------ AUTO -----
     */
    public void initAuto() {
        Pose2d start;
        if (left && isRed) {
            start = new Pose2d(new Vector2d(-36, -62), Math.toRadians(180)); // starting position for red left
        } else {
            start = new Pose2d(new Vector2d(36, 62), Math.toRadians(0));
        }

        mecanum = new Mecanum(this, start);
        lift = new Lift(this);
        intake = new Intake(this);
        mecanum.makeFieldCentric();
        sensors = new SensorPackage(this);
        sensors.enableAprilTagTracking();

        register(mecanum, lift, sensors);

        new SequentialCommandGroup(
                new StrafeToPose(this, new Pose2d(-50, 0, 180))
        ).schedule();
    }
}
