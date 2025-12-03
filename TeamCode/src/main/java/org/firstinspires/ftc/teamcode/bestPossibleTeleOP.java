package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Christopher Gallick's TeleOp")
public class bestPossibleTeleOP extends OpMode {

    // Drive motors (gamepad1)
    private DcMotor frontLeftDrive, frontRightDrive, backLeftDrive, backRightDrive;

    // Mechanism motors (gamepad2)
    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    private DcMotor intake, fly, roler;
    private Servo pushServo, shooterLeft, shooterRight;
    private RevColorSensorV3 color;
    private IMU imu;

    // Utility classes
    Outtake outtake;
    Transfer transfer;
    FlywheelController control;
    // Smooth motor control accel
    private double intakeSpeed = 0;
    private double flySpeed = 0;
    private final double ACCEL = 0.02;

    // Flywheel speed tracking
    private int lastPosition = 0;
    private long lastTime = 0;
    int aprilTagID = 21;

    @Override
    public void init() {
        // DRIVE MOTORS
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "lf");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rf");
        backLeftDrive   = hardwareMap.get(DcMotor.class, "lr");
        backRightDrive  = hardwareMap.get(DcMotor.class, "rr");

        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // MECHANISMS
        intake = hardwareMap.get(DcMotor.class, "intake");
        fly    = hardwareMap.get(DcMotor.class, "fly");
        roler  = hardwareMap.get(DcMotor.class, "roler");
        pushServo = hardwareMap.get(Servo.class, "pushServo");
        shooterLeft = hardwareMap.get(Servo.class, "shooterLeft");
        shooterRight = hardwareMap.get(Servo.class, "shooterRight");

        color = hardwareMap.get(RevColorSensorV3.class, "color");
        imu   = hardwareMap.get(IMU.class, "imu");

        outtake = new Outtake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        control = new FlywheelController(hardwareMap, gamepad2, telemetry);

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                );
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        lastTime = System.nanoTime();

        telemetry.addData("Status", "Ready 🍌🔥");
        telemetry.update();

        initAprilTag();
    }

    @Override
    public void loop() {
        telemetryAprilTag();
        // ---------- DRIVE CONTROL ----------
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x / 2;

        frontLeftDrive.setPower(y + x + turn);
        frontRightDrive.setPower(y - x - turn);
        backLeftDrive.setPower(y - x + turn);
        backRightDrive.setPower(y + x - turn);

        // ---------- INTAKE CONTROL ----------
        double targetIntake = gamepad2.left_stick_y;
        if (Math.abs(targetIntake) < 0.05) targetIntake = 0.0;
        intakeSpeed += Math.signum(targetIntake - intakeSpeed) * ACCEL;
        intake.setPower(intakeSpeed);
        // ---------- FLYWHEEL CONTROL ----------
//        double targetFly = gamepad2.right_stick_y;
//        if (Math.abs(targetFly) < 0.05) targetFly = 0.0;
//        flySpeed += Math.signum(targetFly - flySpeed) * ACCEL;
//        fly.setPower(flySpeed);

        control.update();
        // ---------- ROLLER CONTROL ----------
        if (gamepad2.x) roler.setPower(1);
        else if (gamepad2.a) roler.setPower(-1);
        else if (gamepad2.y) roler.setPower(0);

        // ---------- TRANSFER CONTROL ----------
        Transfer.updateServos(gamepad2.left_trigger, gamepad2.right_trigger);

        // ---------- SHOOTER SERVOS ----------
        if (gamepad2.right_bumper) {
            shooterLeft.setPosition(1.0);
            shooterRight.setPosition(0.0);
        }
        if (gamepad2.left_bumper) {
            shooterLeft.setPosition(0.0);
            shooterRight.setPosition(1.0);
        }

        // ---------- INDEXER AUTO COLOR DETECT ----------
        int r = color.red();
        int g = color.green();
        int b = color.blue();

        float[] hsv = new float[3];
        android.graphics.Color.RGBToHSV(r, g, b, hsv);
        float hue = hsv[0];

        // Debug telemetry for color sensor
        telemetry.addData("Red", r);
        telemetry.addData("Green", g);
        telemetry.addData("Blue", b);
        telemetry.addData("Hue", hue);

        if (hue >= 150 && hue <= 165) {
            pushServo.setPosition(0.65); // green 0.6 works
        } else if (hue >= 210 && hue <= 230) {
            pushServo.setPosition(0.35); // purple 0.4 works
        } else {
            pushServo.setPosition(0.5); // neutral
        }

        // ---------- FLYWHEEL SPEED TELEMETRY ----------
        int currentPos = fly.getCurrentPosition();
        long currentTime = System.nanoTime();

        double dt = (currentTime - lastTime) / 1e9;
        double dPos = currentPos - lastPosition;

        double ticksPerSec = dPos / dt;
        double RPM = (ticksPerSec / 28.0) * 60.0;

        lastPosition = currentPos;
        lastTime = currentTime;

        if (gamepad1.a) {
            Outtake.autoAimingAnglearseAzul();
        }
        telemetry.addData("Fly Speed (RPM)", (int) RPM);
        telemetry.addData("Fly Power", fly.getPower());
        telemetry.addData("Intake Power", intake.getPower());
        telemetry.addData("Roller Power", roler.getPower());
        telemetry.addLine("TeleOp Running 🚀");
        telemetry.update();
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
                aprilTagID = detection.id;
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }
}