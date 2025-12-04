package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class FlywheelController {

    private final DcMotorEx flyMotor;
    private final Gamepad gamepad;

    private double targetRPM = 0;
    private double power = 0.0;

    // Tune these two values
    private final double kUp = 0.003;   // how fast to increase power
    private final double kDown = 0.0001; // how fast to decrease power

    public FlywheelController(HardwareMap hardwareMap, Gamepad gamepad) {
        this.gamepad = gamepad;
        this.flyMotor = hardwareMap.get(DcMotorEx.class, "fly");

        flyMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    public void update() {

        // RPM presets
        if (gamepad.dpad_up) targetRPM = 4000;
        else if (gamepad.dpad_down) targetRPM = 3000;
        else if (gamepad.dpad_left) targetRPM = 2700;
        else if (gamepad.dpad_right) targetRPM = 4800;
        // Measure current RPM
        double currentRPM = flyMotor.getVelocity() * 60.0 / 28.0;

        // Control logic ONLY when stick is up
        if (gamepad.right_stick_y < -0.1) {

            if (currentRPM < targetRPM) {
                power += kUp;
            } else {
                power -= kDown;
            }

            // Clamp
            power = Math.max(0, Math.min(1, power));

            flyMotor.setPower(power);

        } else {
            // Stick not pushed – turn off
            flyMotor.setPower(0);
            power = 0; // reset ramp
        }
    }

    public double getCurrentRPM() {
        return flyMotor.getVelocity() * 60.0 / 28.0;
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public double getPower() {
        return power;
    }
}
