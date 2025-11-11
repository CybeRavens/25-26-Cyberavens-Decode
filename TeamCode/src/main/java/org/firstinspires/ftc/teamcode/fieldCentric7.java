package org.firstinspires.ftc.teamcode;

import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
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

import java.util.List;

@TeleOp
public class fieldCentric7 extends LinearOpMode {
    DcMotor fL, fR, bL, bR, fW, iT;
    NormalizedColorSensor colorSensor;

    private CRServo transferServoGreen, transferServoPurple;

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

        transferServoGreen = hardwareMap.get(CRServo.class, "transfer_servo");
        transferServoPurple = hardwareMap.get(CRServo.class, "purple");
        transferServoGreen.setDirection(DcMotorSimple.Direction.FORWARD);
        transferServoPurple.setDirection(DcMotorSimple.Direction.REVERSE);

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

    }

    private void intake() {
        iT.setPower(1);


    }

    private void autoAiming() {

    }

}


