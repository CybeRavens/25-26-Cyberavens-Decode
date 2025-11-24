package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
@TeleOp
@Disabled
public class fieldCentric5 extends LinearOpMode {

    // ---- Drive Motors ----
    DcMotor fL, fR, bL, bR;

    // ---- Servo ----
    Servo msJackson;

    // ---- Sensors ----
    IMU imu;
    GoBildaPinpointDriver odo;

    // ---- Vision ----
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // ---- Servo Tuning ----
    private static final double KP_BEARING = 0.015; // proportional gain for smooth tracking
    private static final double SERVO_MIN = 0.0;
    private static final double SERVO_MAX = 1.0;
    private static final double SERVO_CENTER = 0.5; // neutral facing forward

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------------- Hardware Map ----------------
        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");

        msJackson = hardwareMap.get(Servo.class, "msJackson");
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        imu = hardwareMap.get(IMU.class, "imu");

        // ---------------- IMU Setup ----------------
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        // ---------------- Motor Directions ----------------
        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------------- Vision Setup ----------------
        initAprilTag();

        telemetry.addLine("Initialized — waiting for start...");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // ---------------- Main Loop ----------------
        while (opModeIsActive()) {

            // Drive robot field-centrically
            fieldCentricDrive();

            // Keep camera servo pointed at tag
            trackTagBearing();

            // Display tag telemetry
            telemetryAprilTag();

            telemetry.update();
        }

        if (visionPortal != null) visionPortal.close();
    }

    // ---------------- FIELD CENTRIC DRIVE ----------------
    private void fieldCentricDrive() {
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
    }

    // ---------------- APRILTAG CAMERA TRACKING ----------------
    private void trackTagBearing() {
        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections.isEmpty()) return;

        AprilTagDetection tag = detections.get(0);

        // Skip any detections missing pose or metadata
        if (tag.ftcPose == null || tag.metadata == null) {
            telemetry.addLine("Tag detected but pose data not ready yet.");
            return;
        }

        double bearing = tag.ftcPose.bearing; // in degrees

        // Smooth proportional adjustment
        double servoAdjust = KP_BEARING * bearing;
        double newPos = SERVO_CENTER - servoAdjust; // negative = turn toward tag

        newPos = Math.max(SERVO_MIN, Math.min(SERVO_MAX, newPos));
        msJackson.setPosition(newPos);

        telemetry.addData("Tag Bearing (deg)", "%.2f", bearing);
        telemetry.addData("Servo Position", "%.3f", newPos);
    }

    // ---------------- APRILTAG TELEMETRY ----------------
    private void telemetryAprilTag() {
        if (aprilTag == null) return;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        telemetry.addData("# Tags", detections.size());

        for (AprilTagDetection detection : detections) {
            if (detection.ftcPose != null && detection.metadata != null) {
                telemetry.addLine(String.format("ID %d | range=%.1f in | bearing=%.1f°",
                        detection.id,
                        detection.ftcPose.range,
                        detection.ftcPose.bearing));
            } else {
                telemetry.addLine(String.format("ID %d | Pose not available yet", detection.id));
            }
        }
    }

    // ---------------- APRILTAG INITIALIZATION ----------------
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
}
