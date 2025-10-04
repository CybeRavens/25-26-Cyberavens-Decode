/*
this code will use the strafe odo pos to change msJackson serv
msJackson servo is the bottom servo plugged into port 2 on the controll hub
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.lang.Math;

import java.util.List;

@TeleOp
public class fieldCentric4 extends LinearOpMode {
    DcMotor fL, fR, bL, bR;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo computer, msJackson;
    public double servoPos = 0.5; // change to whatever is lowest
    GoBildaPinpointDriver odo;


    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();

        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");

        computer = hardwareMap.get(Servo.class, "computer"); // <-- changed to Servo
        msJackson = hardwareMap.get(Servo.class, "msJackson"); // <-- changed to Servo

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        while (opModeInInit()) {
            telemetryAprilTag();
            boolean tagSeen = !aprilTag.getDetections().isEmpty();

            while (!tagSeen) {
                tagSeen = !aprilTag.getDetections().isEmpty();
                servoPos += 0.010;
                computer.setPosition(servoPos);
                if (servoPos >= 0.65) {
                    telemetry.addData("Status", "Not found: breaking");
                    break;
                }
                sleep(500);
            }

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            telemetryAprilTag();


            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

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
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);


            boolean tagSeen = !aprilTag.getDetections().isEmpty();

            telemetry.addData("IMU (radians): ", botHeading);
            telemetry.addData("y", y);
            telemetry.addData("x", x);
            telemetry.addData("rx", rx);
            telemetry.addData("Servo (computer)", tagSeen ? "Locked (Tag Seen)" : "Searching");
            telemetry.update();
        }
        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }
    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            servoPos = (detection.center.y > 355) ? servoPos - 0.010 : servoPos + 0.010;
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void turnBasedOnXY(double aprilTagCenterX, double aprilTagCenterY) {
        // need to change
        double kp = 0.8;
        double ki = 0.05;
        double kd = 0.2;

        double integral = 0.0;
        double prevError = 0.0;

        double phiMin = -90.0;
        double phiMax = 90.0;

        double thetaMount = 0.0;

        double odoX = odo.getPosX(DistanceUnit.INCH);
        double odoY = odo.getPosY(DistanceUnit.INCH);
        double heading = odo.getHeading(AngleUnit.DEGREES);

        //finds turning angle
        double thetaTagRad = Math.atan2((aprilTagCenterY - odoY), (aprilTagCenterX - odoX));
        double thetaTagDeg = Math.toDegrees(thetaTagRad);

        double phiDes = (thetaTagDeg - heading);
        phiDes = wrapAngle(phiDes);

    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < 180) angle += 360;
        return angle;
    }
}
