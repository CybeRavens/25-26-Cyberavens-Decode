package org.firstinspires.ftc.teamcode.pedroPathing;

import static android.os.SystemClock.sleep;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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

@Autonomous(name = "Rad Rupali's path", group = "Autonomous")
@Configurable
public class rickyMomTest extends OpMode {

    private static final boolean USE_WEBCAM = true;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private int pathState;
    private Paths paths;
    private int aprilTagID = -1;  // default: no tag id
    private DcMotor roler, fly;

    // 🚀 Make Transfer global so shooterLogic can access it
    private Transfer transfer;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        Intake intake = new Intake(hardwareMap);
        Outtake outtake = new Outtake(hardwareMap);
        transfer = new Transfer(hardwareMap);

        roler = hardwareMap.get(DcMotor.class, "roler");
        fly = hardwareMap.get(DcMotor.class, "fly");
        paths = new Paths(follower);
        initAprilTag();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update();
        pathState = autonomousPathUpdate();
        Intake.index();

        fly.setPower(-0.7);

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {
        public PathChain Path1, Path2, Path3;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(71.697, 7.579), new Pose(72.000, 43.655)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(72.000, 43.655), new Pose(50.779, 98.072)))
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(135))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(
                            new Pose(50.779, 98.072),
                            new Pose(67.604, 82.762),
                            new Pose(33.347, 83.823)
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(135), Math.toRadians(180))
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
                    // Scan AprilTag for 3 seconds
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
                    shooterLogic(aprilTagID);  // 🔥 Shoot based on detected AprilTag
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    Intake.run(-0.8);
                    roler.setPower(0.7);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    sleep(1000);
                    Intake.stop();
                    roler.setPower(0);
                    telemetry.addLine("done");
                    telemetry.update();
                    pathState++;
                }
                break;
        }
        return pathState;
    }

    // 🎯 SHOOTER LOGIC HERE
    public void shooterLogic(int id) {
        if (id == 21) {
            transfer.updateServos(0,1);
            sleep(5000);
            transfer.updateServos(1,0);
            sleep(5000);
            transfer.updateServos(1,0);
            sleep(5000);

        } else if (id == 22) {
            transfer.updateServos(1,0);
            sleep(5000);
            transfer.updateServos(0,1);
            sleep(5000);
            transfer.updateServos(1,0);
            sleep(5000);

        } else if (id == 23) {
            transfer.updateServos(0,1);
            sleep(5000);
            transfer.updateServos(0,1);
            sleep(5000);
            transfer.updateServos(1,0);
            sleep(5000);
        } else {
            telemetry.addLine("No AprilTag detected → Default shooting");
            telemetry.update();
        }
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
            aprilTagID = detection.id;  // 💾 Store ID for auto shoot
        }
    }
}
