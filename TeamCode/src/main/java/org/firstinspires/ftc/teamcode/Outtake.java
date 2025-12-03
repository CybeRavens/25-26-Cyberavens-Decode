package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Outtake {

    static private DcMotor flyMotor;
    static private GoBildaPinpointDriver pinpoint;
    public static Follower follower;

    public Outtake(HardwareMap hardwareMap) {
        flyMotor = hardwareMap.get(DcMotor.class, "fly");
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        follower = Constants.createFollower(hardwareMap);

        //follower.setStartingPose(new Pose(0, 0, Math.toRadians(90)));


    }

    public static void setVelocity(double power) {
        flyMotor.setPower(power);
    }

    public static void stop() {
        flyMotor.setPower(0);
    }

    public static void autoAimingAnglearseAzul() {
        double xPos = pinpoint.getPosX(DistanceUnit.INCH);
        double yPos = pinpoint.getPosY(DistanceUnit.INCH);
        double heading = pinpoint.getHeading(AngleUnit.DEGREES);
        double goalX = -32;
        double goalY = 110;

        double turnAngle = Math.atan2((goalX - xPos), (goalY-yPos));
        PathChain Path1;

        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(xPos, yPos), new Pose(xPos, yPos))
                )
                .setLinearHeadingInterpolation(Math.toRadians(heading), Math.toRadians(turnAngle))
                .build();

        follower.followPath(Path1, false);

        follower.update();
    }
}
