package org.firstinspires.ftc.teamcode;

import static java.lang.Math.atan2;
import static java.lang.Math.exp;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;


import java.util.List;

@TeleOp
@Disabled
public class fieldCentric7 extends LinearOpMode {
    DcMotor fL, fR, bL, bR, fW, iT;
    ColorSensor colorSensor;

    int aprilTagID = 21;

    private CRServo transferServoGreen, transferServoPurple;
    public Servo shooterAngleServo, pushServo;

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    Follower follower;
    Path goToPoint;


    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();

        fL = hardwareMap.get(DcMotor.class, "lf");
        fR = hardwareMap.get(DcMotor.class, "rf");
        bL = hardwareMap.get(DcMotor.class, "lr");
        bR = hardwareMap.get(DcMotor.class, "rr");

        shooterAngleServo = hardwareMap.get(Servo.class, "shooterAngleServo");

        colorSensor = hardwareMap.get(ColorSensor.class, "color");
        pushServo = hardwareMap.get(Servo.class, "pushServo");


        transferServoGreen = hardwareMap.get(CRServo.class, "green");

        transferServoPurple = hardwareMap.get(CRServo.class, "purple");

        fW = hardwareMap.get(DcMotor.class, "flyWheel");
        iT = hardwareMap.get(DcMotor.class, "intake");

        bL.setDirection(DcMotorSimple.Direction.REVERSE);
        fR.setDirection(DcMotorSimple.Direction.FORWARD);
        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bR.setDirection(DcMotorSimple.Direction.FORWARD);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.DOWN));
        imu.initialize(parameters);

        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(0, 0, 0));

        waitForStart();
        if (isStopRequested()) return;
        fW.setPower(0.7);

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

            if (gamepad1.options) imu.resetYaw();

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX *= 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            fL.setPower(frontLeftPower);
            bL.setPower(backLeftPower);
            fR.setPower(frontRightPower);
            bR.setPower(backRightPower);

            /// auto park
            if (gamepad1.a) {
                Pose currentPose = follower.getPose();
                Pose targetPose = new Pose(20, 20, Math.toRadians(0));

                PathChain goToPoint = follower.pathBuilder()
                        .addPath(new com.pedropathing.geometry.BezierLine(currentPose, targetPose))
                        .build();

                follower.followPath(goToPoint, true);
            }

            if (gamepad1.b) {
                intake();
            }

            if (gamepad1.left_trigger > 0.1) {
                outtake();
            }

            follower.update();

            telemetry.addData("IMU (radians): ", botHeading);
            telemetry.addData("Pose", follower.getPose());
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
        }
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }

    private void outtake() {
        // double sinLa = follower.getPose().getX();
        // double xGoal = (sinLa - 0.0);
        double xGoalPos = 0.0;
        double xRobot = follower.getPose().getX();
        double xGoal = Math.abs(xRobot-xGoalPos);
        final double G = 9.81;
        final double V0 = 23;
        double K = 1;

        double angleRadians = calculateAngle(V0, K, xGoal, G);
        double angleDegrees = Math.toDegrees(angleRadians);
        double servoPos = mapAngleToServo(angleDegrees);

        shooterAngleServo.setPosition(servoPos);

        telemetry.addData("xGoal (m)", xGoal);
        telemetry.addData("Launch Angle (deg)", angleDegrees);
        telemetry.addData("Servo Pos", servoPos);
        telemetry.update();

        if (aprilTagID == 21) {
            fW.setPower(1);
            transferServoGreen.setPower(1);
            transferServoGreen.setPower(0);
            sleep(500);
            transferServoPurple.setPower(1);
            sleep(1000);
            transferServoPurple.setPower(0);
        }
        fW.setPower(0.7);
    }

    private void intake() {
        iT.setPower(0.5);

        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        boolean isDetected = false;

        while (!isDetected) {
            if (green > 200 && green > red && green > blue) {
                telemetry.addData("Color Detected", "Green");
                telemetry.update();
                iT.setPower(0);
                pushServo.setPosition(0.0);
                isDetected = true;
            } else if (red > 150 && blue > 150 && red < 200 && blue < 200 && green < 100) {
                telemetry.addData("Color Detected", "Purple");
                telemetry.update();
                iT.setPower(0);
                pushServo.setPosition(1.0);
                isDetected = true;
            }
        }
        iT.setPower(0);

    }

    private void autoAiming() {
        System.out.println("jafar");
    }



    /// for outake
    public double calculateAngle(double v0, double k, double x_goal, double g) {
        double expTerm = exp(-k * x_goal / v0);
        double bracketTerm = v0 * (1 - expTerm) - (g * v0 / k) * x_goal;
        return atan2(bracketTerm, x_goal);
    }

    public double mapAngleToServo(double angleDegrees) {
        // Example: 0 deg maps to 0.0, 60 deg maps to 1.0 (tune max angle as needed)
        double minAngle = 40.0;
        double maxAngle = 80.0; // Max shooter angle your hardware allows
        double minServo = 0.0;
        double maxServo = 1.0;
        return (angleDegrees - minAngle) / (maxAngle - minAngle) * (maxServo - minServo) + minServo;
    }

}

