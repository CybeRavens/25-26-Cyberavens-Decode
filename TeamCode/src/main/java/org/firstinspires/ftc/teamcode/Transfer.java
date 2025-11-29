package org.firstinspires.ftc.teamcode;

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
    public static void updateServos(float leftTrigger, float rightTrigger) {
        // Green servo controlled by left trigger
        if (leftTrigger > 0.1) fireGreen();
        else stopGreen();

        // Purple servo controlled by right trigger
        if (rightTrigger > 0.1) firePurple();
        else stopPurple();
    }
}