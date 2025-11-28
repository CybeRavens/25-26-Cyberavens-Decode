package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {

    private static DcMotor intakeMotor;
    private static Servo pushServo;
    private static RevColorSensorV3 color;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        pushServo   = hardwareMap.get(Servo.class, "pushServo");
        color       = hardwareMap.get(RevColorSensorV3.class, "color");
    }

    public static void run(double speed) {
        intakeMotor.setPower(speed);
    }

    public static void ejaculate(double speed) {
        intakeMotor.setPower(-speed);
    }

    public static void stop() {
        intakeMotor.setPower(0);
    }
}
