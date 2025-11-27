package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    static DcMotor intakeMotor;
    static Servo pushServo;
    static RevColorSensorV3 color;

    public Intake(HardwareMap hardwareMap) {
        color = hardwareMap.get(RevColorSensorV3.class, "color");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        pushServo = hardwareMap.get(Servo.class, "pushServo");

    }
    public static void run(double speed) {
        intakeMotor.setPower(speed);
    }
    public static void index() {

        int red = color.red();
        int green = color.green();
        int blue = color.blue();

        float hsv[] = new float[3];
        android.graphics.Color.RGBToHSV(red, green, blue, hsv);
        float hue = hsv[0];

        if (hue >= 150 && hue <= 165) {
            pushServo.setPosition(0.2);
        }

        else if (hue >= 210 && hue <= 230) { // wrap-around purple
            pushServo.setPosition(0.8);
        } else {
            pushServo.setPosition(0.5); // neutral
        }

    }

    public static void stop() {
        intakeMotor.setPower(0);
    }
    public static void ejaculate(double speed) {
        intakeMotor.setPower(-speed);
    }

}
