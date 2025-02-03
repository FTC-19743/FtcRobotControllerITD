package org.firstinspires.ftc.teamcode.testCode;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.core.Point;

import java.util.Locale;

@TeleOp(name = "Calibrate CV ", group = "Test Code")
public class CalibrateCV extends LinearOpMode {

    Intake intake;
    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();
    int frameCount = 0;
    OpenCVSampleDetectorV2.FrameData frameData = null;

    public double calculateDistance(Point p1, Point p2) {
        double xDiff = p2.x - p1.x;
        double yDiff = p2.y - p1.y;
        return Math.sqrt(xDiff * xDiff + yDiff * yDiff);
    }

    @SuppressLint("DefaultLocale")
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
                OpenCVSampleDetector.undistort = !OpenCVSampleDetector.undistort;
                //intake.arduPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
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
                intake.sampleDetector.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
            }
            if (gp1.wasXPressed()) {
                intake.sampleDetector.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
            }
            if (gp1.wasBPressed()) {
                intake.sampleDetector.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
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

            if (frameData != null) {
                int targetX, targetY, adjTargetX, adjTargetY;
                targetX = (int) (intake.axonSlider.getPositionEncoder() + intake.xPixelsToTics(frameData.rectCenterXOffset));
                targetY = (int) (intake.extender.getCurrentPosition() + intake.yPixelsToTics(frameData.rectCenterYOffset));

                adjTargetX = (int) (intake.axonSlider.getPositionEncoder() + intake.xPixelsToTics(frameData.adjRectCenterXOffset));
                adjTargetY = (int) (intake.extender.getCurrentPosition()  + intake.yPixelsToTics(frameData.adjRectCenterYOffset));

                telemetry.addLine(String.format("Target X,Y: %d, %d", targetX, targetY));
                telemetry.addLine(String.format("Adjusted Target X,Y: %d, %d", adjTargetX, adjTargetY));
                Point vertices[] = new Point[4];
                frameData.rrect.points(vertices); // get the 4 corners of the rotated rectangle
                int l1 = (int)calculateDistance(vertices[0],vertices[1]);
                int l2 = (int)calculateDistance(vertices[1],vertices[2]);
                telemetry.addLine(String.format("rrect wdith,height,angle: %d, %d %.0f", Math.min(l1, l2), Math.max(l1, l2), frameData.rrect.angle));
            }
            intake.intakeTelemetry();
            if (intake.sampleDetector.sampleAvgs.val != null && intake.sampleDetector.sampleAvgs.val.length > 0) {
                telemetry.addLine("Average HSV: " + ((int) intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[1]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[2]));
                telemetry.addLine("Dot X: " + ((int) intake.sampleDetector.sampleX) + " Dot Y: " + ((int) intake.sampleDetector.sampleY));
            } else {
                telemetry.addLine("No Sample Average");
            }
            telemetry.addLine("Found One?: " + intake.sampleDetector.foundOne.get());
            telemetry.update();
            teamUtil.pause(100);

        }
        intake.closeCV();
    }
}