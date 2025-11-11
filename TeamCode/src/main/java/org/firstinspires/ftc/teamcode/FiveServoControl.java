package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Five Servo Control", group="Examples")
public class FiveServoControl extends LinearOpMode {

    // Declare servos
    private Servo servo1, servo2, servo3, servo4, servo5;

    @Override
    public void runOpMode() {

        // Map servos to configuration names
        servo1 = hardwareMap.get(Servo.class, "servo1");
        servo2 = hardwareMap.get(Servo.class, "servo2");
        servo3 = hardwareMap.get(Servo.class, "servo3");
        servo4 = hardwareMap.get(Servo.class, "servo4");
        servo5 = hardwareMap.get(Servo.class, "servo5");

        // Initialize all servos to a neutral position
        servo1.setPosition(0.5);
        servo2.setPosition(0.5);
        servo3.setPosition(0.5);
        servo4.setPosition(0.5);
        servo5.setPosition(0.5);

        telemetry.addLine("Ready — press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            servo1.setPosition(1);
            sleep(100);
            servo2.setPosition(1);
            sleep(100);
            servo3.setPosition(1);
            sleep(100);
            servo4.setPosition(1);
            sleep(100);
            servo5.setPosition(1);
            sleep(100);

            servo1.setPosition(0);
            sleep(100);
            servo2.setPosition(0);
            sleep(100);
            servo3.setPosition(0);
            sleep(100);
            servo4.setPosition(0);
            sleep(100);
            servo5.setPosition(0);
            sleep(100);
            }

            telemetry.addData("Servo1", servo1.getPosition());
            telemetry.addData("Servo2", servo2.getPosition());
            telemetry.addData("Servo3", servo3.getPosition());
            telemetry.addData("Servo4", servo4.getPosition());
            telemetry.addData("Servo5", servo5.getPosition());
            telemetry.update();
        }
    }

