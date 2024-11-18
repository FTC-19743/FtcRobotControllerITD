package org.firstinspires.ftc.teamcode.testCode;
import android.graphics.Color;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.libs.AdafruitNeoDriverImpl3;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.w8wjb.ftc.AdafruitNeoDriver;

@Config
@TeleOp(name = "NeoPixel Test", group = "Test Code")
public class NeoTest extends LinearOpMode {

    public static final int NUM_PIXELS = 12;
    public static final int BYTES_PER_PIXEL=4; // RGBW neo pixel device
    public static int WHITE = 0;
    public static int RED = 0;
    public static int BLUE = 0;
    public static int GREEN = 0;
    private TeamGamepad gamepadOne = new TeamGamepad();
    AdafruitNeoDriver neopixels;


    public void runOpMode () {

        teamUtil.init(this);
        gamepadOne.initilize(true);

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "neo");
        ((AdafruitNeoDriverImpl3)neopixels).setNumberOfPixelsAndBytesPerPixel(NUM_PIXELS, BYTES_PER_PIXEL);


        waitForStart();
        while(opModeIsActive()) {
            if (gamepadOne.wasRightBumperPressed()) {
                neopixels.fill(Color.argb(WHITE,RED, GREEN, BLUE));
                neopixels.show();
            }
            if (gamepad1.x && gamepadOne.wasDownPressed()) {
                BLUE--;
            }
            if (gamepad1.x && gamepadOne.wasUpPressed()) {
                BLUE++;
            }

            if (gamepad1.b && gamepadOne.wasUpPressed()) {
                RED++;
            }
            if (gamepad1.b && gamepadOne.wasDownPressed()) {
                RED--;
            }

            if (gamepad1.a && gamepadOne.wasUpPressed()) {
                GREEN++;
            }
            if (gamepad1.a && gamepadOne.wasDownPressed()) {
                GREEN--;
            }

            if (gamepad1.y && gamepadOne.wasUpPressed()) {
                WHITE++;
            }
            if (gamepad1.y && gamepadOne.wasDownPressed()) {
                WHITE--;
            }

            if (gamepadOne.wasRightTriggerPressed()) {
                neopixels.fill(Color.argb(0,0, 0, 0));
                neopixels.show();
            }
            gamepadOne.loop();
            telemetry.addLine("Red : " + RED);
            telemetry.addLine("Green : " + GREEN);
            telemetry.addLine("Blue : " + BLUE);
            telemetry.addLine("White : " + WHITE);
            telemetry.update();
        }
    }

}

