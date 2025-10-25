package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Transfer", group = "Sensor")
public class transfer extends LinearOpMode {

    private CRServo transferServoGreen;
    private CRServo transferServoPurple;
    private Servo backServo;
    private DcMotor fly;

    // Keeps track of the current position for the positional servo
    private double backServoPosition = 0.5; // start centered

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize servos and motor
        transferServoGreen = hardwareMap.get(CRServo.class, "transfer_servo");
        transferServoPurple = hardwareMap.get(CRServo.class, "purple");
        backServo = hardwareMap.get(Servo.class, "adjuster");
        fly = hardwareMap.get(DcMotor.class, "fly"); // matches your config name

        // Optional: set directions if needed
        transferServoGreen.setDirection(DcMotorSimple.Direction.FORWARD);
        transferServoPurple.setDirection(DcMotorSimple.Direction.REVERSE);
        fly.setDirection(DcMotorSimple.Direction.FORWARD);

        // Stop motor when starting
        fly.setPower(0);

        waitForStart();

        while (opModeIsActive()) {

            // Control CRServos
            controlCRServo(transferServoGreen, gamepad1.a, gamepad1.b);
            controlCRServo(transferServoPurple, gamepad1.x, gamepad1.y);

            // Control positional servo
            controlPositionalServo(backServo, gamepad1.left_bumper, gamepad1.right_bumper);

            // Control fly motor with left stick Y
            controlFlyMotor();

            telemetry.addData("Back Servo Pos", backServoPosition);
            telemetry.addData("Fly Motor Power", fly.getPower());
            telemetry.update();

            idle(); // optional
        }
    }

    /**
     * Controls a continuous rotation servo based on two buttons.
     */
    private void controlCRServo(CRServo servo, boolean forwardBtn, boolean backwardBtn) {
        if (forwardBtn) {
            servo.setPower(1.0);
        } else if (backwardBtn) {
            servo.setPower(-1.0);
        } else {
            servo.setPower(0);
        }
    }

    /**
     * Controls a standard positional servo with bumpers.
     */
    private void controlPositionalServo(Servo servo, boolean decreaseBtn, boolean increaseBtn) {
        double increment = 0.01; // how much to move per loop

        if (increaseBtn) {
            backServoPosition += increment;
        } else if (decreaseBtn) {
            backServoPosition -= increment;
        }

        // Clamp between 0–1
        backServoPosition = Math.max(0.0, Math.min(1.0, backServoPosition));

        servo.setPosition(backServoPosition);
    }

    /**
     * Controls the fly motor using the left stick Y axis.
     * Pushing up spins one way; pulling down spins the other.
     */
    private void controlFlyMotor() {
        double stickY = gamepad1.left_stick_y;

        if (stickY < -0.2) { // stick pushed up
            fly.setPower(1.0);
        } else if (stickY > 0.2) { // stick pulled down
            fly.setPower(-1.0);
        } else {
            fly.setPower(0);
        }
    }
}
