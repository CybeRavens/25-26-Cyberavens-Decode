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
    public static void index() {
        int r = color.red();
        int g = color.green();
        int b = color.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0];

        if (hue >= 150 && hue <= 165) {
            pushServo.setPosition(0.5); // green 0.6 works
        } else if (hue >= 210 && hue <= 230) {
            pushServo.setPosition(0.4); // purple 0.4 works
        } else {
            pushServo.setPosition(0.45); // neutral
        }
    }
}