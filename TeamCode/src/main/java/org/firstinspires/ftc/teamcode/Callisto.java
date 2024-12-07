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
import org.firstinspires.ftc.teamcode.commands.Drive;
import org.firstinspires.ftc.teamcode.commands.Rotate;
import org.firstinspires.ftc.teamcode.commands.SplineToPose;
import org.firstinspires.ftc.teamcode.commands.StrafeToPose;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeExtensionWithTimout;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByPlayer;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderByTime;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeShoulderDown;
import org.firstinspires.ftc.teamcode.commands.intake.IntakeSpinByTime;
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
        mecanum = new Mecanum(this, new Pose2d(new Vector2d(0,0),0));
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
                new InstantCommand(() -> {mecanum.resetFieldCentricTarget();})
        );

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

        // BUTTON A -- INTAKE RETRACT
        Button aButtonP2 = new GamepadButton(player2, GamepadKeys.Button.A);
        aButtonP2.whenPressed(new InstantCommand(() -> {intake.setExtension(1);}));

        // BUTTON X -- INTAKE EXTEND
        Button xButtonP2 = new GamepadButton(player2, GamepadKeys.Button.X);
       // xButtonP2.whenPressed(new IntakeExtend(this));
        xButtonP2.whenPressed(new InstantCommand(() -> {intake.setExtension(0);}));

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

        //  BUMPER -- sequantial command to put blocks in the basket
        Button leftBumperP2 = new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);
        //leftBumperP2.whenPressed(new IntakeShoulderUp(this));
        //leftBumperP2.whenPressed(new IntakeShoulderByPot(this));
        leftBumperP2.whenPressed(new SequentialCommandGroup(new InstantCommand(() -> {lift.levelBasket();}),
                                                            new IntakeExtensionWithTimout(this, 0, 3000),
                                                            new IntakeShoulderByTime(this, 0.4, 2000),
                                                            new IntakeSpinByTime(this, 2000, 0.3),
                                                            new InstantCommand(() -> {lift.nestBasket();}),
                                                            new IntakeShoulderByTime(this, -0.4, 1500),
                                                            new IntakeExtensionWithTimout(this, 1, 3000),
                                                            new IntakeShoulderByTime(this, 0.4, 2000)));


        // LEFT TRIGGER -- SHOULDER DOWN
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whenActive(new IntakeShoulderDown(this));

        // DPAD DOWN -- LIFT LOWER
        Button downDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);
        downDpadP2.whenPressed(new LiftLowerRTP(this));

        // DPAD LEFT -- RAISE THEN DUMP THEN LOWER
        Button leftDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT);
        leftDpadP2.whenPressed(new SequentialCommandGroup(
                // we always lower first to reset encoder
                new LiftLowerRTP(this),
                new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, true),
                new LiftLowerRTP(this)
        ));

        // DPAD UP -- LIFT RAISE
        Button upDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        upDpadP2.whenPressed(new SequentialCommandGroup(
                // we always lower first to reset encoder
                new LiftLowerRTP(this),
                new LiftRaiseRTP(this, Constants.HIGH_HEIGHT)
        ));

        // DPAD RIGHT -- available
        Button rightDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_RIGHT);

        // RIGHT BUMPER -- NEGATIVE SPIN INTAKE
        Button rightBumperP2 = new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER);
        rightBumperP2.whenHeld(new InstantCommand(()->{
            intake.setSpinSpeed(-0.5);
        }));
        // RIGHT BUMPER RELEASE -- SPIN STOP
        rightBumperP2.whenReleased(new InstantCommand(()->{
            // these servo functions don't know when to stop, so we kill 'em on release
            intake.setSpinSpeed(0);
        }));

        // RIGHT TRIGGER -- SPIN INTAKE
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightTriggerP2.whenActive(new InstantCommand(()->{
            intake.setSpinSpeed(0.5);
        }));
        // RIGHT TRIGGER RELEASE -- SPIN STOP
        rightTriggerP2.whenInactive(new InstantCommand(()->{
            // these servo functions don't know when to stop, so we kill 'em on release
            intake.setSpinSpeed(0.0);
        }));
    }

    /**
     * -----------------
     * ------ AUTO -----
     */
    public void initAuto(){
        Pose2d start;
        if(left) {
            start = new Pose2d(new Vector2d(-15, -65), Math.toRadians(90)); // starting position for red left
            //start = new Pose2d(new Vector2d(0, 0), Math.toRadians(0));
        }
        else {
            start = new Pose2d(new Vector2d(0, 0), 0);
        }

        mecanum = new Mecanum(this, start);
        lift = new Lift(this);
        intake = new Intake(this);
        mecanum.makeRobotCentric();
        sensors = new SensorPackage(this);

        register(mecanum, lift, sensors);

        // LEFT SIDE: GO BIG
        if(left) {
            new SequentialCommandGroup(
                  // new SplineToPose(this, new Pose2d(new Vector2d(24,0 ), 0))
                  // new MoveToPose(this, new Pose2d(new Vector2d(48,-85 ), 0))
                  //  new StrafeToPose(this,new Pose2d(new Vector2d(0,0 ), Math.toRadians(0)))
                 // new MoveToPose(this,new Pose2d(new Vector2d(0,-48 ), Math.toRadians(0)))
               //   new Rotate(this, 135)
                 //   new StrafeToPose(this,new Pose2d(new Vector2d(-48,-60 ),90))
                    new StrafeToPose(this, new Pose2d(new Vector2d(-38,-40), 90)),
                    new ParallelCommandGroup(
                        new Rotate(this, 225),
                        new InstantCommand(()-> {
                            lift.levelBasket();
                        })
                    ),
                    new ParallelCommandGroup(
                        new LiftRaiseThenDump(this, Constants.HIGH_HEIGHT, false),
                        new StrafeToPose(this, new Pose2d(new Vector2d(-60,-62),225)
                        )
                    ),
                    new InstantCommand(()-> {
                        lift.dumpBasket();
                    }),
                    new ParallelCommandGroup(
                        new StrafeToPose(this, new Pose2d(new Vector2d(-26,-44), 135)),
                        new LiftLowerRTP(this)
                    ),
                    new Rotate(this,135)



            ).schedule();
        // RIGHT SIDE: JUST PARK
        } else {
           new StrafeByTime(this, 3.0, 0.25).schedule();
        }

    }
}
