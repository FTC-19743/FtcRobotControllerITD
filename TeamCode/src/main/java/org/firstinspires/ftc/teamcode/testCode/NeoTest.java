package org.firstinspires.ftc.teamcode.testCode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.w8wjb.ftc.AdafruitNeoDriver;


@TeleOp(name = "NeoPixel Test", group = "Test Code")
public class NeoTest extends OpMode {

    private static final int NUM_PIXELS = 30;

    AdafruitNeoDriver neopixels;

    @Override
    public void init() {

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neo");
        neopixels.setNumberOfPixels(NUM_PIXELS);

    }

    public void loop () {
        if (gamepad1.a) {
            // Signal purple pixel
            neopixels.fill(Color.rgb(207, 145, 255));
            neopixels.show();
        } else if (gamepad1.b) {
            // Signal green pixel
            neopixels.fill(Color.rgb(69, 255, 58));
            neopixels.show();
        } else if (gamepad1.x) {
            // Signal yellow pixel
            neopixels.fill(Color.rgb(255, 240, 64));
            neopixels.show();
        } else {
            // Lights off
            neopixels.fill(0);
            neopixels.show();
        }
    }

}

