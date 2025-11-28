package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad2;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {

    private static CRServo transferServoGreen, transferServoPurple;

    public Transfer(HardwareMap hardwareMap) {
        transferServoGreen  = hardwareMap.get(CRServo.class, "green");
        transferServoPurple = hardwareMap.get(CRServo.class, "purple");
    }

    // Fire forward
    public static void fireGreen() {
        transferServoGreen.setPower(1);
    }

    public static void firePurple() {
        transferServoPurple.setPower(-1);
    }

    // Fire backward
    public static void reverseGreen() {
        transferServoGreen.setPower(-1);
    }

    public static void reversePurple() {
        transferServoPurple.setPower(-1);
    }

    // Stop servos
    public static void stopGreen() {
        transferServoGreen.setPower(0);
    }

    public static void stopPurple() {
        transferServoPurple.setPower(0);
    }

    public static void nothing() {
        transferServoGreen.setPower(0);
        transferServoPurple.setPower(0);
    }

    // Update servos based on triggers

}
