package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
public class fullTele extends LinearOpMode {

    // Drive motors
    DcMotor fL, fR, bL, bR;

    // Shooter, intake, roler
    DcMotor fly, intakeMotor, roler;

    // Servos
    CRServo green, purple;
    Servo shooterRight, shooterLeft, pushServo;

    // Color sensor
    RevColorSensorV3 color;

    // Roler running state
    boolean rolerForward = false;

    @Override
    public void runOpMode() {

        // --- Hardware mapping ---
        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");

        fly = hardwareMap.get(DcMotor.class, "fly");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        roler = hardwareMap.get(DcMotor.class, "roler");

        green = hardwareMap.get(CRServo.class, "green");
        purple = hardwareMap.get(CRServo.class, "purple");
        shooterRight = hardwareMap.get(Servo.class, "shooterRight");
        shooterLeft = hardwareMap.get(Servo.class, "shooterLeft");
        pushServo = hardwareMap.get(Servo.class, "pushServo");

        color = hardwareMap.get(RevColorSensorV3.class, "color");

        // Motor directions
        bL.setDirection(DcMotor.Direction.REVERSE);
        fL.setDirection(DcMotor.Direction.REVERSE);

        // IMU
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
        ));
        imu.initialize(parameters);

        waitForStart();

        while (opModeIsActive()) {

            // -------------------------
            // GAMEPAD 1 - Field-centric driving
            // -------------------------
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            if (Math.abs(y) < 0.01) y = 0;
            if (Math.abs(x) < 0.01) x = 0;
            if (Math.abs(rx) < 0.01) rx = 0;

            y = Math.pow(y, 3);
            x = Math.pow(x, 3);
            rx = Math.pow(rx, 3);

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            fL.setPower((rotY + rotX + rx) / denominator);
            bL.setPower((rotY - rotX + rx) / denominator);
            fR.setPower((rotY - rotX - rx) / denominator);
            bR.setPower((rotY + rotX - rx) / denominator);

            // -------------------------
            // GAMEPAD 2 - Flywheel
            // -------------------------
            double flyPower = -gamepad2.left_stick_y;
            if (Math.abs(flyPower) < 0.05) flyPower = 0;
            fly.setPower(flyPower);

            // -------------------------
            // Intake control - right stick
            // -------------------------
            double stick = -gamepad2.right_stick_y;
            if (Math.abs(stick) < 0.1) stick = 0;
            intakeMotor.setPower(stick);

            // -------------------------
            // Roler control - A = start, B = stop
            // -------------------------
            if (gamepad2.a) rolerForward = true;
            if (gamepad2.b) rolerForward = false;
            roler.setPower(rolerForward ? 1 : 0);

            // -------------------------
            // Transfer CR servos - triggers
            // -------------------------
            if (gamepad2.left_trigger > 0.1) {
                green.setPower(0);
                purple.setPower(-1);
            } else if (gamepad2.right_trigger > 0.1) {
                green.setPower(0);
                purple.setPower(1);
            } else {
                green.setPower(0);
                purple.setPower(0);
            }

            // -------------------------
            // Manual CR servo override - bumpers / X/Y
            // -------------------------
            if (gamepad2.left_bumper) green.setPower(1);
            else if (gamepad2.right_bumper) green.setPower(-1);
            else green.setPower(0);

            if (gamepad2.x) purple.setPower(1);
            else if (gamepad2.y) purple.setPower(-1);
            else purple.setPower(0);

            // -------------------------
            // Push servo using hue-based detection
            // -------------------------
            int red = color.red();
            int green = color.green();
            int blue = color.blue();

            float hsv[] = new float[3];
            android.graphics.Color.RGBToHSV(red, green, blue, hsv);
            float hue = hsv[0]; // 0-360 degrees

// Detect green ball (wide range)
            if (hue >= 150 && hue <= 165) {
                pushServo.setPosition(0.2);
                telemetry.addData("Detected Color", "Green");
            }
// Detect purple ball (wide range)
            else if (hue >= 210 && hue <= 230) { // wrap-around purple
                pushServo.setPosition(0.8);
                telemetry.addData("Detected Color", "Purple");
            } else {
                pushServo.setPosition(0.5); // neutral
                telemetry.addData("Detected Color", "None");
            }

            telemetry.addData("Hue", hue);
            telemetry.update();


            // -------------------------
            // Telemetry
            // -------------------------

        }
    }
}
