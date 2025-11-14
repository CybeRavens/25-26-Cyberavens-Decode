package org.firstinspires.ftc.teamcode.pedroPathing;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Sensor: Color with Servo", group = "Sensor")
public class indexTester extends LinearOpMode {

    NormalizedColorSensor colorSensor;
    Servo colorServo; // Added servo
    View relativeLayout;

    @Override
    public void runOpMode() {
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier(
                "RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        try {
            runSample();
        } finally {
            relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.WHITE));
        }
    }

    protected void runSample() {
        float gain = 2;
        final float[] hsvValues = new float[3];
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed;

        // --- HARDWARE SETUP ---
        // Update the name inside quotes to match your configuration file in the Driver Station.
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color"); // <<< UPDATE THIS >>>
        colorServo = hardwareMap.get(Servo.class, "color_servo"); // <<< UPDATE THIS >>>

        // Turns on built-in LED (if your sensor supports it)
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        waitForStart();

        while (opModeIsActive()) {
            // Adjust sensor gain using gamepad buttons
            if (gamepad1.a) {
                gain += 0.005;
            } else if (gamepad1.b && gain > 1) {
                gain -= 0.005;
            }

            colorSensor.setGain(gain);

            // Toggle sensor light with X button
            xButtonCurrentlyPressed = gamepad1.x;
            if (xButtonCurrentlyPressed != xButtonPreviouslyPressed) {
                if (xButtonCurrentlyPressed && colorSensor instanceof SwitchableLight) {
                    SwitchableLight light = (SwitchableLight) colorSensor;
                    light.enableLight(!light.isLightOn());
                }
            }
            xButtonPreviouslyPressed = xButtonCurrentlyPressed;

            // Get current color readings
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
            float hue = hsvValues[0];

            // Display sensor data
            telemetry.addLine()
                    .addData("Red", "%.3f", colors.red)
                    .addData("Green", "%.3f", colors.green)
                    .addData("Blue", "%.3f", colors.blue);
            telemetry.addLine()
                    .addData("Hue", "%.3f", hue)
                    .addData("Saturation", "%.3f", hsvValues[1])
                    .addData("Value", "%.3f", hsvValues[2]);
            telemetry.addData("Alpha", "%.3f", colors.alpha);

            if (colorSensor instanceof DistanceSensor) {
                telemetry.addData("Distance (cm)", "%.3f",
                        ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM));
            }

            // --- SERVO CONTROL BASED ON COLOR DETECTED ---
            // Hue range for purple ~236, green ~157
            if (hue > 220 && hue < 250) { // Purple
                colorServo.setPosition(0.1);// <<< CHANGE THESE VALUES IF NEEDED >>>
                telemetry.addData("Detected Color", "Purple");
            } else if (hue > 140 && hue < 170) { // Green
                colorServo.setPosition(0.9);// <<< CHANGE THESE VALUES IF NEEDED >>>
                telemetry.addData("Detected Color", "Green");
            } else if (hue ==0) {
                colorServo.setPosition(0.5);
                telemetry.addData("No Detected Color", "No Color");
            }
//aman
            telemetry.update();

            // Change RC background to detected color
            relativeLayout.post(() -> relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues)));
        }
    }
}
