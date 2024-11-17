package org.firstinspires.ftc.teamcode.testCode;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.w8wjb.ftc.AdafruitNeoDriver;

@Config
@TeleOp(name = "NeoPixel Test", group = "Test Code")
public class NeoTest extends LinearOpMode {

    private static final int NUM_PIXELS = 30;
    private static int INTENSITY = 10;
    private static int RED_ADJUSTMENT = 0;
    private static int BLUE_ADJUSTMENT = 0;
    private static int GREEN_ADJUSTMENT = 0;
    private TeamGamepad gamepadOne = new TeamGamepad();
    AdafruitNeoDriver neopixels;


    public void runOpMode () {

        teamUtil.init(this);
        gamepadOne.initilize(true);

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neo");
        neopixels.setNumberOfPixels(NUM_PIXELS);


        waitForStart();
        while(opModeIsActive()) {
            if (gamepadOne.wasRightBumperPressed()) {
                // Signal purple pixel
                neopixels.fill(Color.rgb(INTENSITY + RED_ADJUSTMENT, INTENSITY + GREEN_ADJUSTMENT, INTENSITY + BLUE_ADJUSTMENT));
                neopixels.show();
            }
            if (gamepadOne.wasLeftPressed()) {
                RED_ADJUSTMENT++;
            }
            if (gamepadOne.wasRightPressed()) {
                RED_ADJUSTMENT--;
            }
            if (gamepadOne.wasDownPressed()) {
                BLUE_ADJUSTMENT--;
            }
            if (gamepadOne.wasUpPressed()) {
                BLUE_ADJUSTMENT++;
            }
            if (gamepadOne.wasYPressed()) {
                GREEN_ADJUSTMENT++;
            }
            if (gamepadOne.wasAPressed()) {
                GREEN_ADJUSTMENT--;
            }
            if (gamepadOne.wasXPressed()) {
                INTENSITY += 5;
            }
            if (gamepadOne.wasBPressed()) {
                INTENSITY -= 5;
            }
            if (gamepadOne.wasLeftBumperPressed()) {
                neopixels.fill(Color.rgb(0, 0, 0));
            }
            gamepadOne.loop();
            telemetry.addLine("Red Adjustment: " + RED_ADJUSTMENT);
            telemetry.addLine("Green Adjustment: " + GREEN_ADJUSTMENT);
            telemetry.addLine("Blue Adjustment: " + BLUE_ADJUSTMENT);
            telemetry.addLine("Intensity: " + INTENSITY);
            telemetry.update();
        }

        //else {
            // Lights off
          //  neopixels.fill(0);
          //  neopixels.show();
        //}
    }

}

