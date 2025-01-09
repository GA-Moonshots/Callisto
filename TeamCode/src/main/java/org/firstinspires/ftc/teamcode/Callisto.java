package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.AlignToGamePiece;
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.ForwardByTime;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.StrafeToPose;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeExtensionWithTimeout;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByPlayer;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByTime;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderDown;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSpinByTime;
import org.firstinspires.ftc.teamcode.commands.intake.TransferBlock;
import org.firstinspires.ftc.teamcode.commands.lift.LiftRaiseThenDump;
import org.firstinspires.ftc.teamcode.commands.RotateByIMU;
import org.firstinspires.ftc.teamcode.commands.StrafeByTime;
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
        mecanum = new Mecanum(this, startOfTele);
        lift = new Lift(this);
        intake = new Intake(this);
        sensors = new SensorPackage(this);

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

      /*  // BUTTON B -- RESET FIELD CENTRIC TARGET
        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(new InstantCommand(() -> {
            mecanum.resetFieldCentricTarget();
        })); */

        // BUTTON B -- TURN 90
        Button bButtonP1 = new GamepadButton(player1, GamepadKeys.Button.B);
        bButtonP1.whenPressed(new InstantCommand(() -> {
            new RotateByIMU(this, 180, 1.45, 0.385).schedule();
        }));

        // BUTTON X -- TURN 180
        Button xButtonP1 = new GamepadButton(player1, GamepadKeys.Button.X);
        xButtonP1.whenPressed(new InstantCommand(() -> {
            new RotateByIMU(this, 180, 2.92, 0.385).schedule();
        }));

        // BUTTON Y -- Reset to Field centric
        Button yButtonP1 = new GamepadButton(player1, GamepadKeys.Button.Y);
        yButtonP1.whenPressed(
                new InstantCommand(() -> {
                    mecanum.resetFieldCentricTarget();
                })
        );

        Button dPadUpP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_UP);
        Button dPadDownP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_DOWN);
        Button dPadLeftP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_LEFT);
        Button dPadRightP1 = new GamepadButton(player1, GamepadKeys.Button.DPAD_RIGHT);

        dPadRightP1.whenPressed(new StrafeToPose(this, new Pose2d(new Vector2d(-36, -62), Math.toRadians(180))));



        /*
                _                                    __
               (_ )                                /'__`\
         _ _    | |    _ _  _   _    __   _ __    (_)  ) )
        ( '_`\  | |  /'_` )( ) ( ) /'__`\( '__)      /' /
        | (_) ) | | ( (_| || (_) |(  ___/| |       /' /( )
        | ,__/'(___)`\__,_)`\__, |`\____)(_)      (_____/'
        | |                ( )_| |
        (_)                `\___/'-50  -50      */

        // BUTTON A -- INTAKE RETRACT
        Button aButtonP2 = new GamepadButton(player2, GamepadKeys.Button.A);
        aButtonP2.whenPressed(new InstantCommand(() -> {
            intake.setExtension(1);
        }));

        // BUTTON X -- INTAKE
        Button xButtonP2 = new GamepadButton(player2, GamepadKeys.Button.X);
        // xButtonP2.whenPressed(new IntakeExtend(this));
        xButtonP2.whenPressed(new InstantCommand(() -> {
            intake.setExtension(0.5);
        }));

        // BUTTON X -- INTAKE ALL THE WAY
        // xButtonP2.whenPressed(new IntakeExtend(this));
        xButtonP2.whenHeld(new InstantCommand(() -> {
            intake.setExtension(0);
        }));
        xButtonP2.whenReleased(new InstantCommand(() -> {
            intake.setExtension(0.5);
        }));

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

        // DPAD LEFT -- RAISE THEN DUMP THEN LOWER
        Button leftDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT);
        leftDpadP2.whenPressed(new SequentialCommandGroup(
                // we always lower first to reset encoder
                new LiftLowerRTP(this),
                new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                new LiftLowerRTP(this)
        ));

        // DPAD UP -- LIFT RAISE
        Button upDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        upDpadP2.whenPressed(new SequentialCommandGroup(
                // we always lower first to reset encoder
                new LiftLowerRTP(this),
                new LiftRaiseRTP(this, Constants.HIGH_HEIGHT)
        ));

        // DPAD RIGHT -- ASSISTANT
        Button rightDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_RIGHT);
        rightDpadP2.whenPressed(new InstantCommand(() -> {
            intake.toggleAssistant();
        }));

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
        mecanum.makeRobotCentric();
        sensors = new SensorPackage(this);

        register(mecanum, lift, sensors);

        // LEFT SIDE
        if (left) {
            // RED LEFT
            if (isRed) {
                 new SequentialCommandGroup(
                         // idea for the new atonomouse the old one is comented out
                         new InstantCommand(() ->{
                             lift.levelBasket();
                         }),
                         //go to basket
                         new StrafeToPose(this, new Pose2d(new Vector2d(-59.5, -60), Math.toRadians(180))),
                         // rotate to face basket
                         new Rotate(this, 230),
                         //dump the block
                         //new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                         // lower lift
                         //new LiftLowerRTP(this),
                         // move over not to bump  fence
                         new StrafeToPose(this, new Pose2d(new Vector2d(-58, -59), Math.toRadians(230))),
                        //rotate to "face" te second block
                        new Rotate(this, 176),
                        // move to second block
                        new StrafeToPose(this, new Pose2d(new Vector2d(-21.5, -31.5), Math.toRadians(176))),
                         // shoulder down
                         new IntakeShoulderByTime(this, -0.25, 1600),
                         // parallel command where we move forward(to push) and extend the arm
                         new StrafeToPose(this, new Pose2d(new Vector2d(-32, -29), Math.toRadians(176))),
                        new IntakeExtensionWithTimeout(this, 0.5, 1500),
                        //intake block
                        new IntakeSpinByTime(this,3000, 0.35),
                        // transfer
                        new IntakeShoulderByTime(this, 0.5, 2000),
                         new IntakeSpinByTime(this,3000,0.5),
                         new IntakeSpinByTime(this,1000,-0.3)











                        /* // set the assistant to be down
                         new InstantCommand(() -> {
                             intake.setAssistant(1);
                            //level basket
                            lift.levelBasket();
                        }),
                        //go to basket
                        new StrafeToPose(this, new Pose2d(new Vector2d(-59.5, -60), Math.toRadians(180))),
                        // rotate to face basket
                        new Rotate(this, 230),
                        //dump the block
                        new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                        // lower lift
                        new LiftLowerRTP(this),
                         // rotate to face the other block vertically
                         new StrafeToPose(this, new Pose2d(new Vector2d(-58, -59), Math.toRadians(230))),
                         new Rotate(this, 92),
                         // move to pose in front of the second block
                         new StrafeToPose(this, new Pose2d(new Vector2d(-51.25, -58), Math.toRadians(90))),
                         // lower shoulder
                         new IntakeShoulderByTime(this, -0.4, 1000),
                         // extend arm
                         new IntakeExtensionWithTimeout(this, 0.3, 1500),
                         // intake block and move forward
                         new ParallelCommandGroup(
                             new IntakeSpinByTime(this,3000, 0.3),
                             new StrafeToPose(this, new Pose2d(new Vector2d(-51.25, -57), Math.toRadians(90)))
                         ),
                        // lift shoulder and level basket
                         new IntakeShoulderByTime(this, 0.5, 2500),
                         //
                         new IntakeExtensionWithTimeout(this,0.25,500),
                         // lift up the assistant
                         new InstantCommand(() -> {
                             intake.setAssistant(0);
                         }),
                         // level basket
                         new InstantCommand(() -> {
                             lift.levelBasket();
                         }),
                        // spit it out
                        new IntakeSpinByTime(this,3000, -0.3),
                        // retract arm
                         new IntakeExtensionWithTimeout(this, 1.0, 1500),
                        // go to basket
                         new StrafeToPose(this, new Pose2d(new Vector2d(-61, -59), Math.toRadians(180))),
                         // rotate to face basket
                         new Rotate(this, 230),
                         //dump the block
                         new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                         // lower lift
                         new LiftLowerRTP(this)*/
                ).schedule();
            }

            // BLUE LEFT
            else {
                // UNTESTED MIRRORED CODE
                new SequentialCommandGroup(
                        // level basket
                        new InstantCommand(() -> {
                            lift.levelBasket();
                        }),
                        // go to basket (mirrored from -59, -59.5 to 59, 59.5)
                        new StrafeToPose(this, new Pose2d(new Vector2d(59, 59.5), Math.toRadians(0))),
                        // rotate to face basket (230 becomes 130 for blue side)
                        new Rotate(this, 50),
                        // dump the block
                        new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                        // lower lift
                        new LiftLowerRTP(this),

                        // rotate to face other block (98 becomes 262)
                        new Rotate(this, 262),
                        new InstantCommand(() -> {
                            lift.levelBasket();
                        }),
                        // move forward to other block (mirrored from -58, -53.25 to 58, 53.25)
                        new StrafeToPose(this, new Pose2d(new Vector2d(58, 53.25), Math.toRadians(135))), // 225 becomes 135
                        // lower shoulder
                        new IntakeShoulderByTime(this, -0.4, 2500),
                        // extend the arm
                        new IntakeExtensionWithTimeout(this, 0.05, 1000),
                        // intake block
                        new IntakeSpinByTime(this, 200, 0.5),
                        // lift the shoulder
                        new IntakeShoulderByTime(this, 0.4, 2500),
                        // spit it out
                        new IntakeSpinByTime(this, 2000, -0.6),
                        // intake arm
                        new IntakeExtensionWithTimeout(this, 1, 1500),
                        // go to basket (mirrored from -59, -59.5 to 59, 59.5)
                        new StrafeToPose(this, new Pose2d(new Vector2d(59, 59.5), Math.toRadians(0))),
                        // rotate to face basket (230 becomes 130)
                        new Rotate(this, 50),

                        // dump the block
                        new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                        // lower lift
                        new LiftLowerRTP(this)
                ).schedule();
            }
        }
        // RIGHT SIDE
        else {
            //new StrafeByTime(this, 3.0, 0.25).schedule();
            new AlignToGamePiece(this);
        }
    }


}
