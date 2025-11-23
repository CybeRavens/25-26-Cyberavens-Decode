package org.firstinspires.ftc.teamcode;

import static android.os.SystemClock.sleep;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    static DcMotor intakeMotor;
    static Servo pushServo;
    static ColorSensor colorSensor;

    public Intake(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        intakeMotor  = hardwareMap.get(DcMotor.class, "intake");
        pushServo = hardwareMap.get(Servo.class, "pushServo");

    }
    public static void run() {
        intakeMotor.setPower(1);
    }
//    public static void index() {
//
//        int red = colorSensor.red();
//        int green = colorSensor.green();
//        int blue = colorSensor.blue();
//
//        if (green > 200 && green > red && green > blue) {
//            telemetry.addData("Color Detected", "Green");
//            telemetry.update();
//            pushServo.setPosition(0.0);
//        } else if (red > 150 && blue > 150 && red < 200 && blue < 200 && green < 100) {
//            telemetry.addData("Color Detected", "Purple");
//            telemetry.update();
//            pushServo.setPosition(1.0);
//        }
//    }

    public static void stop() {
        intakeMotor.setPower(0);
    }
    public static void ejaculate() {
        intakeMotor.setPower(-1);
    }

}
