package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.telemetry.SelectableOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@Configurable
@TeleOp(name = "Tests", group = "Test")
public class Tests extends SelectableOpMode {

    public Tests() {
        super("Select a Test", s -> {
            s.folder("Drive Motors", f -> {
                f.add("Front Right Motor", FrontRightMotorTest::new);
                f.add("Front Left Motor", FrontLeftMotorTest::new);
                f.add("Back Right Motor", BackRightMotorTest::new);
                f.add("Back Left Motor", BackLeftMotorTest::new);
            });
            s.folder("Servos", f -> {
                f.add("Standard Servo", StandardServoTest::new);
                f.add("CR Servo", CRServoTest::new);
            });
            s.folder("Sensors", f -> {
                f.add("Color Sensor", ColorSensorTest::new);
            });
        });
    }

    /** Shared utility to initialize drive motors */
    public static void initMotors(LinearOpMode opMode, DcMotor[] motors) {
        motors[0] = opMode.hardwareMap.get(DcMotor.class, "lf");
        motors[1] = opMode.hardwareMap.get(DcMotor.class, "rf");
        motors[2] = opMode.hardwareMap.get(DcMotor.class, "lr");
        motors[3] = opMode.hardwareMap.get(DcMotor.class, "rr");

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE); // bL
        motors[1].setDirection(DcMotorSimple.Direction.FORWARD); // fR
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE); // fL
        motors[3].setDirection(DcMotorSimple.Direction.FORWARD); // bR
    }
}

// ---------- Motor Tests ----------
class FrontRightMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0]; fR = motors[1]; bL = motors[2]; bR = motors[3];

        waitForStart();
        while (opModeIsActive()) fR.setPower(0.5);
    }
}

class FrontLeftMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0]; fR = motors[1]; bL = motors[2]; bR = motors[3];

        waitForStart();
        while (opModeIsActive()) fL.setPower(0.5);
    }
}

class BackRightMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0]; fR = motors[1]; bL = motors[2]; bR = motors[3];

        waitForStart();
        while (opModeIsActive()) bR.setPower(0.5);
    }
}

class BackLeftMotorTest extends LinearOpMode {
    DcMotor fL, fR, bL, bR;

    @Override public void runOpMode() {
        DcMotor[] motors = new DcMotor[4];
        Tests.initMotors(this, motors);
        fL = motors[0]; fR = motors[1]; bL = motors[2]; bR = motors[3];

        waitForStart();
        while (opModeIsActive()) bL.setPower(0.5);
    }
}

// ---------- Servo Tests ----------
class StandardServoTest extends LinearOpMode {
    private Servo testServo;

    @Override public void runOpMode() {
        testServo = hardwareMap.get(Servo.class, "xyautoaim");

        telemetry.addLine("Standard Servo Test Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) testServo.setPosition(0.0);
            else if (gamepad1.b) testServo.setPosition(1.0);

            telemetry.addData("Servo Position", testServo.getPosition());
            telemetry.update();
        }
    }
}

class CRServoTest extends LinearOpMode {
    private CRServo testCRServo;

    @Override public void runOpMode() {
        testCRServo = hardwareMap.get(CRServo.class, "computer");

        telemetry.addLine("CR Servo Test Ready");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.a) testCRServo.setPower(1.0);
            else if (gamepad1.b) testCRServo.setPower(-1.0);
            else testCRServo.setPower(0.0);

            telemetry.addData("CRServo Power", testCRServo.getPower());
            telemetry.update();
        }
    }
}

// ---------- Sensor Test ----------
class ColorSensorTest extends LinearOpMode {
    private NormalizedColorSensor colorSensor;

    @Override public void runOpMode() {
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
        int red = Color.red(argb);
        int green = Color.green(argb);
        int blue = Color.blue(argb);

        int max = Math.max(red, Math.max(green, blue));
        if (max == 0) max = 1;

        red   = (red   * 255) / max;
        green = (green * 255) / max;
        blue  = (blue  * 255) / max;

        return new int[]{red, green, blue};
    }
}
