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

@Autonomous(name = "Brad's rad path", group = "Autonomous")
@Configurable // Panels
public class autov67 extends OpMode {

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
        follower.setStartingPose(new Pose(64.724, 9.398, Math.toRadians(90)));

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


        Intake.index();
        fly.setPower(-0.75);
        //roler.setPower(0.5);
        Intake.run(-0.3);

    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(64.724, 9.398), new Pose(72.303, 48.202))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(72.303, 48.202), new Pose(62, 91))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(130))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(62, 91),
                                    new Pose(84.126, 80.640),
                                    new Pose(40, 84.278)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(130), Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40, 84.278), new Pose(50.627, 99.587))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(130))
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
                    pathState++;
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    shooterLogic(aprilTagID);
                    sleep(3000);
                    pathState++;
                }
                break;

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
            transfer.firePurple();
            sleep(4000);
            roler.setPower(1);
            Intake.run(-1);
            transfer.fireGreen();
            sleep(2000);
            transfer.nothing();
            sleep(2000);
            transfer.fireGreen();

        } else if (id == 22) {
            transfer.fireGreen();
            sleep(1000);
            transfer.firePurple();
            sleep(1000);
            transfer.fireGreen();
            sleep(1000);

        } else if (id == 23) {
            transfer.fireGreen();
            sleep(1000);
            transfer.firePurple();
            sleep(1000);
            transfer.firePurple();
            sleep(1000);
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


