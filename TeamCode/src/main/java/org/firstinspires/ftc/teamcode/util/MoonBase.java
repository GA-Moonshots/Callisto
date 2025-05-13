package org.firstinspires.ftc.teamcode.util;

import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Callisto;

public class MoonBase extends SubsystemBase{

    // COMMON INSTANCE VARIABLES
    protected Callisto robot;
    protected Telemetry telemetry;
    public MoonBase(Callisto robot) {
        this.robot = robot;
        this.telemetry = robot.telemetry;
    }
}
