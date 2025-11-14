package org.firstinspires.ftc.teamcode;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    static private CRServo transferServoGreen, transferServoPurple;

    public Transfer(HardwareMap hardwareMap) {
        transferServoGreen  = hardwareMap.get(CRServo.class, "green");
        transferServoPurple = hardwareMap.get(CRServo.class, "purple");
    }

    public static void fireGreen() {
        transferServoGreen.setPower(-1);
    }

    public static void firePurple() {
        transferServoPurple.setPower(1);
    }

    public static void nothing() {
        transferServoGreen.setPower(0);
        transferServoGreen.setPower(0);
    }
}
