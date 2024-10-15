package org.firstinspires.ftc.teamcode.testCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

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
        intake.initCV();


        telemetry.addLine("Ready to start");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()){
            if (gp1.wasLeftBumperPressed()) {
                intake.setExposure();
                intake.setGain();
            }
            if (gp1.wasRightBumperPressed()) {
                teamUtil.log("Pixel HSV: " + ((int)intake.sampleDetector.samplePixel.val[0]) + ", " + ((int)intake.sampleDetector.samplePixel.val[1]) + ", "+ ((int)intake.sampleDetector.samplePixel.val[2]));
                teamUtil.log("Average HSV: " + ((int)intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int)intake.sampleDetector.sampleAvgs.val[1]) + ", "+ ((int)intake.sampleDetector.sampleAvgs.val[2]));
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
            telemetry.addLine("Pixel HSV: " + ((int)intake.sampleDetector.samplePixel.val[0]) + ", " + ((int)intake.sampleDetector.samplePixel.val[1]) + ", "+ ((int)intake.sampleDetector.samplePixel.val[2]));
            telemetry.addLine("Average HSV: " + ((int)intake.sampleDetector.sampleAvgs.val[0]) + ", " + ((int)intake.sampleDetector.sampleAvgs.val[1]) + ", "+ ((int)intake.sampleDetector.sampleAvgs.val[2]));
        }
    }
}
