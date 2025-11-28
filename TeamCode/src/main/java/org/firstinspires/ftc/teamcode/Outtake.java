package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {

    static private DcMotor flyMotor;

    public Outtake(HardwareMap hardwareMap) {
        flyMotor = hardwareMap.get(DcMotor.class, "fly");
    }

    public static void setVelocity(double power) {
        flyMotor.setPower(power);
    }

    public static void stop() {
        flyMotor.setPower(0);
    }
}
