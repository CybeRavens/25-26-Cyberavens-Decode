package org.firstinspires.ftc.teamcode.pedroPathing;

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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous - 11/22", group = "Autonomous")
@Configurable // Panels
public class autoV5 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(40.168, 123, Math.toRadians(270)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.168, 125), new Pose(48.354, 95.343))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(120))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.354, 95.343),
                                    new Pose(55.478, 80.185),
                                    new Pose(23.949, 83.823)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(23.949, 83.823), new Pose(55, 96))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.354, 95.343),
                                    new Pose(63.360, 54.568),
                                    new Pose(25.9, 60.025)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(25.9, 60.025), new Pose(55, 96))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.505, 95.192),
                                    new Pose(46.686, 63.360),
                                    new Pose(30, 70.029)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(30, 70.029),
                                    new Pose(86.855, 32.741),
                                    new Pose(24.253, 35.773)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(24.253, 35.773), new Pose(55, 96))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(155))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55, 95.343), new Pose(53, 34.257))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(120), Math.toRadians(90))
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
                    follower.followPath(paths.Path2, true);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path3, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path4, true);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path5, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path6, true);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path7, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path8, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.Path9, true);
                    pathState++;
                }
                break;
        }
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}