package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "ANI backRight", group = "ANI")

public class backRight extends LinearOpMode{
    DcMotor fL;

    @Override public void runOpMode() {

        fL = hardwareMap.get(DcMotor.class, "rr");
        waitForStart();
        while (opModeIsActive()) {
            fL.setPower(0.5);
        }
    }
}