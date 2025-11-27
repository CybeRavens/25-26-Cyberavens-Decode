package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    static private Servo shooterAngleServoLeft, shooterAngleServoRight;
    //static private DcMotorEx flyMotor;
    static private DcMotor flyMotor;

    public Outtake(HardwareMap hardwareMap) {
        shooterAngleServoLeft  = hardwareMap.get(Servo.class, "shooterLeft");
        shooterAngleServoRight = hardwareMap.get(Servo.class, "shooterRight");
        flyMotor = hardwareMap.get(DcMotor.class, "fly");
        //flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public static void setAngle(double theta) {
        shooterAngleServoLeft.setPosition(300/theta);
        shooterAngleServoRight.setPosition(300/(1-theta));
    }
    public static void setVelocity(float v) {
        //flyMotor.setVelocity(v);
        flyMotor.setPower(v);
    }

    public static void stop() {
        flyMotor.setPower(0);
    }
}
