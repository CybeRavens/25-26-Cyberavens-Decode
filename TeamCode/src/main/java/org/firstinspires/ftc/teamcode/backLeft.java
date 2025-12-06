package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ANI BreakPad", group = "ANI")

public class backLeft extends LinearOpMode{
    Servo fL;

    @Override public void runOpMode() {

        fL = hardwareMap.get(Servo.class, "pushServo");
        waitForStart();
        while (opModeIsActive()) {
            fL.setPosition(0.5);
        }
    }
}