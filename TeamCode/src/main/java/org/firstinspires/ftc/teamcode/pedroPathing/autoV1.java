package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

@Autonomous(name = "PedroPathing_AutoV1_Positive", group = "Autonomous")
public class autoV1 extends LinearOpMode {
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
    private final Pose p1  = new Pose(57, -34, Math.toRadians(90));
    private final Pose p2  = new Pose(0, 0, Math.toRadians(0));
    private final Pose p3  = new Pose(57, -34, Math.toRadians(90));
    private final Pose p4  = new Pose(0, 0, Math.toRadians(0));
    private final Pose p5  = new Pose(57, -34, Math.toRadians(90));
    private final Pose p6  = new Pose(0, 0, Math.toRadians(0));
    private final Pose p7  = new Pose(57, -34, Math.toRadians(90));

    private PathChain pathChain;

    @Override
    public void runOpMode() throws InterruptedException {

        if (follower == null) follower = Constants.createFollower(hardwareMap);
        if (telemetryM == null) telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        if (Tuning.poseHistory == null) Tuning.poseHistory = follower.getPoseHistory();

        follower.setStartingPose(startPose);

        pathChain = follower.pathBuilder()
                .addPath(new BezierLine(startPose, p1))
                .addPath(new BezierLine(p1, p2))
                .addPath(new BezierLine(p2, p3))
                //.addPath(new BezierLine(p3, p4))
                //.addPath(new BezierLine(p4, p5))
                //.addPath(new BezierLine(p5, p6))
                //.addPath(new BezierLine(p6, p7))
                .build();

        while (!isStarted() && !isStopRequested()) {
            follower.update();
            drawCurrent();

        }

        waitForStart();
        if (isStopRequested()) return;

        follower.followPath(pathChain, true);

        while (opModeIsActive()) {
            follower.update();
            drawCurrentAndHistory();
            telemetry.addData("X", follower.getPose().getX());
            telemetry.addData("Y", follower.getPose().getY());
            telemetry.addData("Heading (deg)", Math.toDegrees(follower.getPose().getHeading()));
            telemetry.update();
        }
    }
}
