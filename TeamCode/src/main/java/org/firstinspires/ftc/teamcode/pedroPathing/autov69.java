package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Outtake;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

import java.util.List;

@Autonomous(name = "Red Far Side", group = "Autonomous")
@Configurable // Panels
public class autov69 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private int aprilTagID = -1;  // default: no tag id
    private DcMotor roler, fly;
    private Transfer transfer;
    private Intake intake;
    private Outtake outtake;
    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(144-56, 8.5, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        fly = hardwareMap.get(DcMotor.class, "fly");
        roler = hardwareMap.get(DcMotor.class, "roler");


        initAprilTag();
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);


        //Intake.index();
        fly.setPower(-0.9);
        //roler.setPower(0.5);
        Intake.run(-0.3);

    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(144-56.000, 8.500), new Pose(144-60.000, 8.500)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180-80))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-60.000, 8.500), new Pose(144-60.000, 10))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180-80), Math.toRadians(60))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(144-60.000, 10), new Pose(144-59.088, 50.824))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(60), Math.toRadians(90))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.Path1, true);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    ElapsedTime timer = new ElapsedTime();
                    timer.reset();
                    while (timer.seconds() < 3.0) {
                        telemetryAprilTag();
                        telemetry.update();
                    }
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    shooterLogic(aprilTagID);
                    sleep(3000);
                    Transfer.nothing();
                    Intake.stop();
                    fly.setPower(0);
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
        }
        return pathState;
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
        telemetry.addData("# Tags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("ID", detection.id);
            aprilTagID = detection.id;
        }
    }


    public void shooterLogic(int id) {
        //fly.setPower(-0.7);
        if (id == 21) {
            sleep(1000);
            transfer.firePurple();
            sleep(4000);
            roler.setPower(1);
            Intake.run(-1);
            transfer.fireGreen();
            sleep(2000);
            //transfer.nothing();
            sleep(2000);
            transfer.fireGreen();
            sleep(5000);
            Intake.stop();
            roler.setPower(0);

        } else if (id == 22) {
            transfer.fireGreen();
            sleep(3500);
            transfer.nothing();
            transfer.firePurple();
            sleep(3000);
            transfer.nothing();
            roler.setPower(1.0);
            Intake.run(-1);
            sleep(2000);
            transfer.fireGreen();
            sleep(6000);
            roler.setPower(0);
            Intake.stop();


        } else if (id == 23) {
            transfer.fireGreen();
            sleep(2000);
            roler.setPower(1.0);
            transfer.nothing();
            Intake.run(-1);
            sleep(3000);
            transfer.fireGreen();
            sleep(4000);
            transfer.firePurple();
            sleep(5000);
            roler.setPower(0);
            Intake.stop();
        } else {
            transfer.firePurple();
            sleep(4000);
            roler.setPower(1);
            Intake.run(-1);
            transfer.fireGreen();
            sleep(2000);
            transfer.nothing();
            sleep(2000);
            transfer.fireGreen();
        }
    }
}


