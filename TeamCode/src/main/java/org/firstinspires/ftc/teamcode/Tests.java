package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

/**
 * Utility class for initializing drive motors with the correct directions.
 * Used by all motor test OpModes.
 */
public class Tests {
    public static void initMotors(LinearOpMode opMode, DcMotor[] motors) {
        motors[0] = opMode.hardwareMap.get(DcMotor.class, "lf"); // front left
        motors[1] = opMode.hardwareMap.get(DcMotor.class, "rf"); // front right
        motors[2] = opMode.hardwareMap.get(DcMotor.class, "lr"); // back left
        motors[3] = opMode.hardwareMap.get(DcMotor.class, "rr"); // back right

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE); // bL
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD); // fR
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE); // fL
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD); // bR
    }
}

/**
 * Test OpMode for the front right motor.
 */
@TeleOp(name = "Motor Test - Front Right", group = "Tests")
class FrontRightMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0];
        fR = motors[1];
        bL = motors[2];
        bR = motors[3];

        waitForStart();
        while (opModeIsActive()) {
            fR.setPower(0.5);
        }
    }
}

/**
 * Test OpMode for the front left motor.
 */
@TeleOp(name = "Motor Test - Front Left", group = "Tests")
class FrontLeftMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0];
        fR = motors[1];
        bL = motors[2];
        bR = motors[3];

        waitForStart();
        while (opModeIsActive()) {
            fL.setPower(0.5);
        }
    }
}

/**
 * Test OpMode for the back right motor.
 */
@TeleOp(name = "Motor Test - Back Right", group = "Tests")
class BackRightMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0];
        fR = motors[1];
        bL = motors[2];
        bR = motors[3];

        waitForStart();
        while (opModeIsActive()) {
            bR.setPower(0.5);
        }
    }
}

/**
 * Test OpMode for the back left motor.
 */
@TeleOp(name = "Motor Test - Back Left", group = "Tests")
class BackLeftMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override
    public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0];
        fR = motors[1];
        bL = motors[2];
        bR = motors[3];

        waitForStart();
        while (opModeIsActive()) {
            bL.setPower(0.5);
        }
    }
}

/**
 * Test OpMode for a standard Servo named "xyautoaim".
 * - Press A to move it to position 0.0.
 * - Press B to move it to position 1.0.
 */
@TeleOp(name = "Servo Test - Standard", group = "Tests")
class StandardServoTest extends LinearOpMode {
    private Servo testServo;

    @Override
    public void runOpMode() {
        testServo = hardwareMap.get(Servo.class, "xyautoaim");

        telemetry.addLine("Standard Servo Test Ready");
        telemetry.addData("Initial Position", testServo.getPosition());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                testServo.setPosition(0.0);
            } else if (gamepad1.b) {
                testServo.setPosition(1.0);
            }

            telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.update();
        }
    }
}

/**
 * Test OpMode for a Continuous Rotation Servo (CRServo) named "computer".
 * - Press A to spin forward.
 * - Press B to spin backward.
 * - Otherwise it stops.
 */
@TeleOp(name = "Servo Test - CRServo", group = "Tests")
class CRServoTest extends LinearOpMode {
    private CRServo testCRServo;

    @Override
    public void runOpMode() {
        testCRServo = hardwareMap.get(CRServo.class, "computer");

        telemetry.addLine("CRServo Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                testCRServo.setPower(1.0); // forward
            } else if (gamepad1.b) {
                testCRServo.setPower(-1.0); // backward
            } else {
                testCRServo.setPower(0.0); // stop
            }

            telemetry.addData("CRServo Power", testCRServo.getPower());
            telemetry.update();
        }
    }
}

/**
 * Test OpMode for a Color Sensor named "color".
 * Reads RGB values, scales them, and displays them on telemetry.
 */
@TeleOp(name = "Color Sensor Test", group = "Tests")
class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "color");

        telemetry.addLine("Color Sensor Test Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int[] rgb = getScaledRGB255();

            telemetry.addData("Red", rgb[0]);
            telemetry.addData("Green", rgb[1]);
            telemetry.addData("Blue", rgb[2]);
            telemetry.update();
        }
    }

    private int[] getScaledRGB255() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        int argb = colors.toColor();

        int red   = Color.red(argb);
        int green = Color.green(argb);
        int blue  = Color.blue(argb);

        // Normalize so the brightest channel = 255
        int max = Math.max(red, Math.max(green, blue));
        if (max == 0) max = 1;

        red   = (red   * 255) / max;
        green = (green * 255) / max;
        blue  = (blue  * 255) / max;

        return new int[]{red, green, blue};
    }
}
