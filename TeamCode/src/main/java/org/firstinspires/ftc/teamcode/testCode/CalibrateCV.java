package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@TeleOp(name = "Calibrate CV ", group = "Test Code")
public class CalibrateCV extends LinearOpMode {

    Intake intake;
    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();


    public void runOpMode() {
        teamUtil.init(this);
        intake = new Intake();
        telemetry.addLine("Initializing");
        telemetry.update();
        gp1.initilize(true);
        gp2.initilize(false);
        intake.initialize();
        intake.initCV(true);


        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            gp1.loop();
            gp2.loop();

            if (gp1.wasLeftBumperPressed()) {
                intake.sampleDetector.configureCam(intake.arduPortal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
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

            intake.intakeTelemetry();
            if (intake.sampleDetector.sampleAvgs.val != null && intake.sampleDetector.sampleAvgs.val.length > 0) {
                telemetry.addLine("Average HSV: " + ((int) intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[1]) + ", " + ((int) intake.sampleDetector.sampleAvgs.val[2]));
            } else {
                telemetry.addLine("No Sample Average");
            }
            telemetry.update();
            teamUtil.pause(100);

        }
    }
}
/*
            if (intake.sampleDetector.samplePixel != null) {
                telemetry.addLine("Pixel HSV: " + ((int) intake.sampleDetector.samplePixel[0]) + ", " + ((int) intake.sampleDetector.samplePixel[1]) + ", " + ((int) intake.sampleDetector.samplePixel[2]));
            } else {
                telemetry.addLine("No Pixel Sample");
            }
            if (intake.sampleDetector.samplePixel.val != null) {
                telemetry.addLine("Pixel HSV: " + ((int) intake.sampleDetector.samplePixel.val[0]) + ", " + ((int) intake.sampleDetector.samplePixel.val[1]) + ", " + ((int) intake.sampleDetector.samplePixel.val[2]));
            } else {
                telemetry.addLine("No Pixel Sample");
            }

 */