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
import java.util.List;

@TeleOp
public class fieldCentric4 extends LinearOpMode {
    DcMotor fL, fR, bL, bR;
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private Servo computer, msJackson;
    public double servoPos = 0.5;
    GoBildaPinpointDriver odo;
    private double integral = 0.0;
    private double prevError = 0.0;
    private double aprilTagY = 0;
    private double aprilTagX = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();
        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");
        computer = hardwareMap.get(Servo.class, "computer");
        msJackson = hardwareMap.get(Servo.class, "msJackson");
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
                servoPos += 0.0003;
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
            //computer.setPosition(servoPos);

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            y = Math.abs(y) < 0.01 ? 0 : Math.pow(y, 3);
            x = Math.abs(x) < 0.01 ? 0 : Math.pow(x, 3);
            rx = Math.abs(rx) < 0.01 ? 0 : Math.pow(rx, 3);

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            rotX *= 1.1;
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower  = (rotY + rotX + rx) / denominator;
            double backLeftPower   = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower  = (rotY + rotX - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);
            if (!aprilTag.getDetections().isEmpty()) {
                AprilTagDetection tag = aprilTag.getDetections().get(0);
                turnBasedOnXY(aprilTagX, aprilTagY);
            }
            telemetry.addData("IMU (rad):", botHeading);
            telemetry.addData("Servo (computer)", !aprilTag.getDetections().isEmpty() ? "Locked (Tag Seen)" : "Searching");
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
        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", detections.size());
        for (AprilTagDetection detection : detections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detection.center.x, detection.center.y));
            }
            aprilTagY = detection.center.y;
            aprilTagX = detection.center.x;
            servoPos = (detection.center.y > 95) ? servoPos - 0.010 : servoPos + 0.010;
        }
    }

    private void turnBasedOnXY(double aprilTagX, double aprilTagY) {
        double Kp = 0.8;
        double Ki = 0.05;
        double Kd = 0.2;

        double phiMin = -90.0;
        double phiMax = 90.0;
        double thetaMount = 0.0;

        double odoX = odo.getPosX(DistanceUnit.INCH);
        double odoY = odo.getPosY(DistanceUnit.INCH);
        double heading = odo.getHeading(AngleUnit.DEGREES);
        double thetaTagRad = Math.atan2(aprilTagY - odoY, aprilTagX - odoX);
        double thetaTagDeg = Math.toDegrees(thetaTagRad);
        double phiDes = wrapAngle(thetaTagDeg - heading - thetaMount);
        double phiCur = msJackson.getPosition();
        double error = wrapAngle(phiDes - phiCur);
        double dt = 0.02;
        integral += error * dt;
        double derivative = (error - prevError) / dt;
        double u = Kp * error + Ki * integral + Kd * derivative;
        prevError = error;

        double phiCmd = clamp(phiCur + u, phiMin, phiMax);
        msJackson.setPosition(phiCmd);
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
