package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;

public class FlywheelController {

    private DcMotorEx flyMotor;
    private Gamepad gamepad;

    private double targetRPM = 0;

    public FlywheelController(HardwareMap hardwareMap, Gamepad gamepad) {
        this.flyMotor = hardwareMap.get(DcMotorEx.class, "fly");
        this.gamepad = gamepad;
    }

    public void update() {
        // Set target RPM based on button press
        if (gamepad.a) {
            targetRPM = 4000;
        } else if (gamepad.b) {
            targetRPM = 3000;
        } else if (gamepad.y) {
            targetRPM = 2700;
        }

        // Get current motor RPM
        double currentRPM = flyMotor.getVelocity() * 60 / (flyMotor.getMotorType().getTicksPerRev());

        // Scale power to stay under target RPM
        double power = flyMotor.getPower(); // current power
        if (currentRPM < targetRPM) {
            power += 0.01; // ramp up
        } else if (currentRPM > targetRPM) {
            power -= 0.01; // ramp down
        }

        // Clamp power to 0..1
        power = Math.max(0, Math.min(1, power));
        flyMotor.setPower(power);
    }
}
