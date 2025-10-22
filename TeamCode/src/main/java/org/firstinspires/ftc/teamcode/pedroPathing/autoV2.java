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

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class autoV2 extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(100.433, 9.0, Math.toRadians(90)));

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

        public PathChain line1;
        public PathChain line2;
        public PathChain line3;
        public PathChain line4;
        public PathChain line5;
        public PathChain line6;
        public PathChain line7;
        public PathChain line8;
        public PathChain line9;
        public PathChain line10;
        public PathChain line11;

        public Paths(Follower follower) {
            line1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(100.433, 9.000), new Pose(84.455, 84.692))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                    .build();

            line2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.455, 84.692),
                                    new Pose(72.612, 43.742),
                                    new Pose(101.307, 35.169)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                    .build();

            line3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.307, 35.169), new Pose(124.753, 35.168))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.753, 35.168),
                                    new Pose(82.936, 37.968),
                                    new Pose(85.404, 84.455)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            line5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.404, 84.455),
                                    new Pose(85.560, 56.515),
                                    new Pose(101.832, 59.490)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            line6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(101.832, 59.490), new Pose(124.578, 59.665))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(124.578, 59.665),
                                    new Pose(94.309, 50.916),
                                    new Pose(85.404, 84.692)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            line8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(85.404, 84.692),
                                    new Pose(83.635, 84.860),
                                    new Pose(102.532, 83.111)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();

            line9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(102.532, 83.111), new Pose(125.803, 83.286))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            line10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(125.803, 83.286),
                                    new Pose(100.083, 71.388),
                                    new Pose(84.692, 84.217)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                    .build();

            line11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(84.692, 84.217),
                                    new Pose(77.337, 23.621),
                                    new Pose(108.890, 18.979)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(paths.line1, true);
                pathState++;
                break;

            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line2, true);
                    pathState++;
                }
                break;

            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line3, true);
                    pathState++;
                }
                break;

            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line4, true);
                    pathState++;
                }
                break;

            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line5, true);
                    pathState++;
                }
                break;

            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line6, true);
                    pathState++;
                }
                break;

            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line7, true);
                    pathState++;
                }
                break;

            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line8, true);
                    pathState++;
                }
                break;

            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line9, true);
                    pathState++;
                }
                break;

            case 9:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line10, true);
                    pathState++;
                }
                break;

            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(paths.line11, true);
                    pathState++;
                }
                break;

        }

        return pathState;
    }

}
