package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
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
import org.firstinspires.ftc.teamcode.commands.ForwardByTime;
import org.firstinspires.ftc.teamcode.commands.IntakeExtend;
import org.firstinspires.ftc.teamcode.commands.IntakeRetract;
//import org.firstinspires.ftc.teamcode.commands.IntakeShoulderByTime;
import org.firstinspires.ftc.teamcode.commands.IntakeShoulderByPlayer;
import org.firstinspires.ftc.teamcode.commands.IntakeShoulderDown;
import org.firstinspires.ftc.teamcode.commands.IntakeShoulderToggle;
import org.firstinspires.ftc.teamcode.commands.IntakeShoulderUpBangBang;
import org.firstinspires.ftc.teamcode.commands.LiftLower;
import org.firstinspires.ftc.teamcode.commands.LiftRaise;
import org.firstinspires.ftc.teamcode.commands.RotateByIMU;
//import org.firstinspires.ftc.teamcode.subsystems.Blinkin;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Lift;
import org.firstinspires.ftc.teamcode.subsystems.Mecanum;
import org.firstinspires.ftc.teamcode.subsystems.SensorPackage;
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

   // public Blinkin blinkin;
    
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
        //blinkin = new Blinkin(this);
        mecanum = new Mecanum(this, new Pose2d(new Vector2d(0,0),0));
        lift = new Lift(this);
        intake = new Intake(this);
        sensors = new SensorPackage(this);

        // Register subsystems
        // REGISTER THE SUBSYSTEM BEFORE THE DEFAULT COMMANDS
        register(mecanum,lift, intake, intake, sensors);

        // Setting Default Commands
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

        // BUTTON A -- INTAKE RETRACT
        Button aButtonP2 = new GamepadButton(player2, GamepadKeys.Button.A);
        aButtonP2.whenPressed(new IntakeRetract(this));

        // BUTTON X -- INTAKE EXTEND
        Button xButtonP2 = new GamepadButton(player2, GamepadKeys.Button.X);
        xButtonP2.whenPressed(new IntakeExtend(this));

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

        // LEFT BUMPER -- SHOULDER TOGGLE
        Button leftBumperP2 = new GamepadButton(player2, GamepadKeys.Button.LEFT_BUMPER);
        leftBumperP2.whenPressed(new IntakeShoulderUpBangBang(this));

        // LEFT TRIGGER -- SPIN INTAKE
        Trigger leftTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        leftTriggerP2.whenActive(new IntakeShoulderDown(this));


        // DPAD DOWN -- LIFT LOWER
        Button downDpadP2 = new GamepadButton(player2, GamepadKeys.Button.DPAD_DOWN);
        downDpadP2.whenPressed(new LiftLower(this, 0));

        // DPAD LEFT -- LIFT RAISE HALF WAY
        Button leftDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_LEFT);
        leftDpadP2.whenPressed(new LiftRaise(this, 150));

        // DPAD UP -- LIFT RAISE
        Button upDpadP2 =  new GamepadButton(player2, GamepadKeys.Button.DPAD_UP);
        upDpadP2.whenPressed(new SequentialCommandGroup(
                new LiftLower(this,0),
                new LiftRaise(this, 2000)
        ));

        // RIGHT BUMPER -- NEGATIVE SPIN INTAKE
        Button rightBumperP2 = new GamepadButton(player2, GamepadKeys.Button.RIGHT_BUMPER);
        rightBumperP2.whenHeld(new InstantCommand(()->{
            intake.setSpinSpeed(-0.5);
        }));
        // RIGHT BUMPER RELEASE -- SPIN STOP
        rightBumperP2.whenReleased(new InstantCommand(()->{
            intake.setSpinSpeed(0);
        }));

        // RIGHT TRIGGER -- SPIN INTAKE
        Trigger rightTriggerP2 = new Trigger(() -> player2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
        rightTriggerP2.whenActive(new InstantCommand(()->{
            intake.setSpinSpeed(0.5);
        }));
        // RIGHT TRIGGER RELEASE -- SPIN STOP
        rightTriggerP2.whenInactive(new InstantCommand(()->{
            intake.setSpinSpeed(0.0);
        }));
    }

    /**
     * -----------------
     * ------ AUTO -----
     */
    public void initAuto(){
        Pose2d start;
        Pose2d end;

        start = new Pose2d(new Vector2d(0,0), 0.0);
        end = new Pose2d(new Vector2d(24,0), 0.0);

        mecanum = new Mecanum(this, start);
        lift = new Lift(this);
        mecanum.makeRobotCentric();
        sensors = new SensorPackage(this);

        register(mecanum, lift, sensors);

        new SequentialCommandGroup(
           new InstantCommand(() -> {
               lift.levelBasket(); }) ,
           new ForwardByTime(this, 2, 0.25),
           new RotateByIMU(this,130, 3.7, 0.27),
           new ForwardByTime(this, 2.25, 0.33),
           //new LiftRaiseAndDump(this, 500, 5),
           new ForwardByTime(this, 2.25, -0.33),
           new RotateByIMU(this, 30, 1.0,0.30),
           new ForwardByTime(this, 1.0, -0.20)
        ).schedule();
    }
}
