package org.firstinspires.ftc.teamcode.testCode;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.w8wjb.ftc.AdafruitNeoDriver;

@Config
@TeleOp(name = "NeoPixel Test", group = "Test Code")
public class NeoTest extends OpMode {

    TeamGamepad gamepad1;

    private static final int NUM_PIXELS = 30;
    private static int INTENSITY = 10;
    private static int RED_ADJUSTMENT = 0;
    private static int BLUE_ADJUSTMENT = 0;
    private static int GREEN_ADJUSTMENT = 0;


    AdafruitNeoDriver neopixels;

    @Override
    public void init() {

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neo");
        neopixels.setNumberOfPixels(NUM_PIXELS);

    }

    public void loop () {
        if (gamepad1.wasRightBumperPressed()) {
            // Signal purple pixel
            neopixels.fill(Color.rgb(INTENSITY+RED_ADJUSTMENT, INTENSITY+GREEN_ADJUSTMENT, INTENSITY+BLUE_ADJUSTMENT));
            neopixels.show();
        }
        if (gamepad1.wasLeftPressed()) {
            RED_ADJUSTMENT++;
        }
        if (gamepad1.wasRightPressed()) {
            RED_ADJUSTMENT--;
        }
        if(gamepad1.wasDownPressed()){
            BLUE_ADJUSTMENT--;
        }
        if (gamepad1.wasUpPressed()){
            BLUE_ADJUSTMENT++;
        }
        if(gamepad1.wasYPressed()){
            GREEN_ADJUSTMENT++;
        }
        if(gamepad1.wasAPressed()){
            GREEN_ADJUSTMENT--;
        }
        if(gamepad1.wasXPressed()){
            INTENSITY+=5;
        }
        if(gamepad1.wasBPressed()){
            INTENSITY-=5;
        }
        if(gamepad1.wasLeftPressed()){
            neopixels.fill(Color.rgb(0, 0, 0));
        }

        telemetry.addLine("Red Adjustment: " + RED_ADJUSTMENT);
        telemetry.addLine("Green Adjustment: " + GREEN_ADJUSTMENT);
        telemetry.addLine("Blue Adjustment: " + BLUE_ADJUSTMENT);
        telemetry.addLine("Intensity: " + INTENSITY);
        telemetry.update();

        //else {
            // Lights off
          //  neopixels.fill(0);
          //  neopixels.show();
        //}
    }

}

