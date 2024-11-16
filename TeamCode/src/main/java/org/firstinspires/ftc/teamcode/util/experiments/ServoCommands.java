package org.firstinspires.ftc.teamcode.util.experiments;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.Callisto;

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
