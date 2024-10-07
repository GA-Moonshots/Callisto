package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.subsystems.ServoTest;

public class ServoCommands extends CommandBase {
    private final Callisto robot;
    private final ServoTest servo;
    public ServoCommands(Callisto robot){
      this.robot = robot;
      this.servo = robot.servo;

      addRequirements(robot.servo);
    }

    @Override
    public void initialize(){


    }

    @Override
    public void execute(){
        if(servo.isOpen){
            servo.close();
        }else{
            servo.open();
        }

        servo.isOpen =!servo.isOpen;

        this.isFinished();
    }

    @Override
    public boolean isFinished(){
        return true;

    }

}
