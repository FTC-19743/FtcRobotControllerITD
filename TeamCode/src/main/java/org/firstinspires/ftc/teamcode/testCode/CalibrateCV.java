package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.Locale;

@TeleOp(name = "Calibrate CV ", group = "Test Code")
public class CalibrateCV extends LinearOpMode {

    Intake intake;
    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();
    int frameCount = 0;
    OpenCVSampleDetector.FrameData frameData = null;


    public void runOpMode() {
        teamUtil.init(this);
        intake = new Intake();
        telemetry.addLine("Initializing");
        telemetry.update();
        gp1.initilize(true);
        gp2.initilize(false);
        intake.initialize();
        intake.initCV(true);
        intake.startCVPipeline();



        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        OpenCVSampleDetector.FrameData newFrame;
        while (opModeIsActive()){
            gp1.loop();
            gp2.loop();

            // Get updated frame data from detector if available
            if (intake.sampleDetector.frameDataQueue.peek()!=null) {
                frameData = intake.sampleDetector.frameDataQueue.peek();
            }

            if (gp1.wasLeftBumperPressed()) {
                intake.sampleDetector.configureCam(intake.arduPortal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }
            if (gp1.wasHomePressed()) {
                intake.arduPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
            }

            if (gp1.wasRightBumperPressed()) {
                teamUtil.log("Average HSV: " + ((int)intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int)intake.sampleDetector.sampleAvgs.val[1]) + ", "+ ((int)intake.sampleDetector.sampleAvgs.val[2]));
                //teamUtil.log("Pixel HSV: " + ((int)intake.sampleDetector.samplePixel[0]) + ", " + ((int)intake.sampleDetector.samplePixel[1]) + ", "+ ((int)intake.sampleDetector.samplePixel[2]));
                //teamUtil.log("Pixel HSV: " + ((int)intake.sampleDetector.samplePixel.val[0]) + ", " + ((int)intake.sampleDetector.samplePixel.val[1]) + ", "+ ((int)intake.sampleDetector.samplePixel.val[2]));
            }
            if (gp1.wasUpPressed()) {
                intake.sampleDetector.sampleUp(gp1.gamepad.right_trigger>0.5? 50: 5);
            }
            if (gp1.wasDownPressed()) {
                intake.sampleDetector.sampleDown(gp1.gamepad.right_trigger>0.5? 50: 5);
            }
            if (gp1.wasLeftPressed()) {
                intake.sampleDetector.sampleLeft(gp1.gamepad.right_trigger>0.5? 50: 5);
            }
            if (gp1.wasRightPressed()) {
                intake.sampleDetector.sampleRight(gp1.gamepad.right_trigger>0.5? 50: 5);
            }
            if (gp1.wasYPressed()) {
                intake.sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
            }
            if (gp1.wasXPressed()) {
                intake.sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
            }
            if (gp1.wasBPressed()) {
                intake.sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
            }
            if (gp1.wasAPressed()) {
                intake.sampleDetector.nextView();
            }
            if(gp1.wasRightTriggerPressed()){
                intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL, true);
            }
            if(gp1.wasLeftTriggerPressed()){
                intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL, false);
            }

            intake.intakeTelemetry();
            if (intake.sampleDetector.sampleAvgs.val != null && intake.sampleDetector.sampleAvgs.val.length > 0) {
                telemetry.addLine("Average HSV: " + ((int) intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[1]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[2]));
                telemetry.addLine("Dot X: " + ((int) intake.sampleDetector.sampleX) + " Dot Y: " + ((int) intake.sampleDetector.sampleY));
            } else {
                telemetry.addLine("No Sample Average");
            }
            telemetry.update();
            teamUtil.pause(100);

        }
        intake.closeCV();
    }
}