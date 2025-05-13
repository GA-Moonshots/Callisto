package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.Callisto;
import org.firstinspires.ftc.teamcode.util.Constants;
import org.firstinspires.ftc.teamcode.util.MoonBase;

public class Intake extends MoonBase {


    public Servo extensionServo;
    public CRServo spinServo;



    // Motor for the shoulder
    public DcMotorEx shoulderMotor;

    public boolean isExtended = false;

    public Intake(Callisto robot) {
        super(robot);
        shoulderMotor = robot.hardwareMap.get(DcMotorEx.class, Constants.SHOULDER_MOTOR_NAME);
        shoulderMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set up the servos
        extensionServo = robot.hardwareMap.get(Servo.class, Constants.EXTEND_INTAKE_SERVO);
        spinServo = robot.hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, Constants.SPIN_INTAKE_SERVO);


        extensionServo.setPosition(1);
    }

    public boolean isExtended() {
        return extensionServo.getPosition() == 0;
    }

    public boolean isRetracted() {
        // TODO: guess; assuming 1 is retracted
        return extensionServo.getPosition() == 1;
    }

    public boolean isUp() {
        return shoulderMotor.getCurrentPosition() > -50;
    }

    public boolean isNearUp() {
        return shoulderMotor.getCurrentPosition() > -100;
    }

    public boolean isNearDown() {
        return shoulderMotor.getCurrentPosition() < -275;
    }

    public boolean isDown() {
        return shoulderMotor.getCurrentPosition() < -325;
    }

    public void setExtension(double position) {
        extensionServo.setPosition(position);
    }

    public void setSpinSpeed(double speed) {
        telemetry.addData("Is it Spinning", speed != 0);
        spinServo.setPower(speed);
    }

    public void stopSpin() {
        spinServo.setPower(0.0);
    }



    // Method to get the current shoulder position
    public double getShoulderPosition() {
        return shoulderMotor.getCurrentPosition();
    }

    public void stopShoulder() {
        shoulderMotor.setPower(0);
    }

    // Accessor method to calculate the shoulder angle using the potentiometer
    public double calculateVoltageByAngle(double angle) {

        // https://docs.revrobotics.com/rev-crossover-products/sensors/potentiometer/application-examples
        return (445.5 * (angle - 270)) / (Math.pow(angle, 2) - (270 - angle) - 36450);
    }

    @Override
    public void periodic() {

        telemetry.addData("Intake Servo", extensionServo.getPosition());

        String shoulderState = isUp() ? "Up"
                : isNearUp() ? "Near Up"
                : isDown() ? "Down"
                : isNearDown() ? "Near Down"
                : "Unknown";
        if(extensionServo != null)
            telemetry.addData("Intake: ", "Extension: " + extensionServo.getPosition() + ", Shoulder: " + shoulderState);
        // Output the current position of the shoulder motor
   //     if(shoulderMotor != null)
           // robot.telemetry.addData("Shoulder Position", getShoulderPosition());

       // robot.telemetry.addData(("Shoulder Angle:"), getShoulderPosition());
    }
}
