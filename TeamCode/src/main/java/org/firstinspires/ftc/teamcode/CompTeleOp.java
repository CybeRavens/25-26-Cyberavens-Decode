package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@TeleOp
public class CompTeleOp extends LinearOpMode {
    DcMotor fL, fR, bL, bR, roler;

    @Override
    public void runOpMode() throws InterruptedException {

        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");
        roler = hardwareMap.get(DcMotor.class, "roler");
        // Reverse necessary motors
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);
        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;
            //deadzone below
            y = Math.abs(y) < 0.01 ? 0 : y;
            x = Math.abs(x) < 0.01 ? 0 : x;
            rx = Math.abs(rx) < 0.01 ? 0 : rx;

            y = Math.pow(y, 3);
            x = Math.pow(x, 3);
            rx = Math.pow(rx, 3);

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            //rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);
            //Intake.index();
            //roler.setPower(1);

            if (gamepad1.a) {
                Intake.run();
            }
            if (gamepad1.b) {
                Intake.stop();
            }
            if (gamepad1.x) {
                Intake.ejaculate();
            }

            if (gamepad1.left_trigger > 0.1) {
                Transfer.firePurple();
            } else if (gamepad1.right_trigger > 0.1) {
                Transfer.fireGreen();
            } else {
                Transfer.nothing();
            }
            if (gamepad1.right_bumper) {
                Outtake.setVelocity(6900);
            }

        }
    }
}

