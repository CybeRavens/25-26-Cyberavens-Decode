package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    static private Servo shooterAngleServoLeft, shooterAngleServoRight;
    static private DcMotorEx flyMotor;

    public Outtake(HardwareMap hardwareMap) {
        shooterAngleServoLeft  = hardwareMap.get(Servo.class, "shooterLeft");
        shooterAngleServoRight = hardwareMap.get(Servo.class, "shooterRight");
        flyMotor = hardwareMap.get(DcMotorEx.class, "flyWheel");

        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public static void setAngle(double theta) {
        shooterAngleServoLeft.setPosition(300/theta);
        shooterAngleServoRight.setPosition(300/(1-theta));
    }
    public static void setVelocity(int v) {
        flyMotor.setVelocity(v);
    }
}
