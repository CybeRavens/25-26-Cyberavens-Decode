package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class FlywheelController {

    private final DcMotorEx flyMotor;
    private final Gamepad gamepad;
    private final Telemetry telemetry;

    // Motor / gearing constants
    private static final double MOTOR_TPR = 389.2;   // encoder ticks per motor rev
    private static final double GEAR_RATIO = 3.0;    // flywheel RPM = motorRPM * GEAR_RATIO
    private static final double MOTOR_MAX_RPM = 1620.0;

    // Flywheel preset caps (wheel side)
    private double flywheelRPMCap = 0; // 0 = unlimited

    // Target flywheel RPM (wheel side)
    private double targetFlywheelRPM = 0;

    // Feedback constants
    private static final double POWER_MIN = 0.05; // minimum power to overcome static friction
    private static final double POWER_MAX = 1.0;
    private static final double DEADZONE = 0.05;

    private double currentPower = 0.0;

    public FlywheelController(HardwareMap hardwareMap, Gamepad gamepad, Telemetry telemetry) {
        this.flyMotor = hardwareMap.get(DcMotorEx.class, "fly");
        this.gamepad = gamepad;
        this.telemetry = telemetry;

        this.flyMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        this.flyMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void update() {
        // --- D-pad selects flywheel cap ---
        if (gamepad.dpad_up) flywheelRPMCap = 3500;
        else if (gamepad.dpad_right) flywheelRPMCap = 2700;
        else if (gamepad.dpad_down) flywheelRPMCap = 2400;
        else if (gamepad.dpad_left) flywheelRPMCap = 10000;

        // --- Stick input ---
        double stick = -gamepad.right_stick_y; // up is positive
        double stickScale = Math.abs(stick) > DEADZONE ? Math.max(0, Math.min(1.0, stick)) : 0.0;

        // Compute desired RPM based on stick
        double maxFlywheelRPM = MOTOR_MAX_RPM * GEAR_RATIO;
        targetFlywheelRPM = stickScale * maxFlywheelRPM;

        // Apply preset cap
        if (flywheelRPMCap > 0 && targetFlywheelRPM > flywheelRPMCap) {
            targetFlywheelRPM = flywheelRPMCap;
        }

        // --- Compute actual flywheel RPM ---
        double actualMotorRPM = flyMotor.getVelocity() * 60.0 / MOTOR_TPR;
        double actualFlywheelRPM = actualMotorRPM * GEAR_RATIO;

        // --- Feedback power calculation ---
        double errorRPM = targetFlywheelRPM - actualFlywheelRPM;

        // Dynamic ramping: larger error → larger adjustment
        double ramp = 0.0005 * errorRPM; // adjust this constant for responsiveness
        currentPower += ramp;

        // Clamp power to min/max
        currentPower = Math.max(POWER_MIN, Math.min(POWER_MAX, currentPower));

        // Stop motor if stick near zero
        if (stickScale <= 0.01) currentPower = 0.0;

        flyMotor.setPower(currentPower);

        // --- Telemetry ---
        telemetry.addData("Requested RPM", (int) targetFlywheelRPM);
        telemetry.addData("Actual RPM", (int) actualFlywheelRPM);
        telemetry.addData("Cap", flywheelRPMCap == 0 ? "UNLIMITED" : (int) flywheelRPMCap);
        telemetry.addData("Motor Power", String.format("%.2f", currentPower));
        telemetry.update();
    }
}
