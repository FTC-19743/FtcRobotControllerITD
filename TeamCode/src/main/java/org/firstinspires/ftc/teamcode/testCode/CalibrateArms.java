package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Calibrate Arms", group = "Test Code")
public class CalibrateArms extends LinearOpMode {

    Intake intake;

    public enum Ops {Intake_Manual_Operation,
        Test_Intake_Speeds,
        Test_Intake_Run_To_Position,
        };
    public static Ops AA_Operation = Ops.Intake_Manual_Operation;



    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();

    private void doAction() {
        switch (AA_Operation) {
            case Intake_Manual_Operation : intakeManualOperation();break;
            case Test_Intake_Speeds : testIntakeSpeeds();break;
            case Test_Intake_Run_To_Position : ;break;

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.SIDE=teamUtil.Side.SCORE;

        intake = new Intake();
        intake.initialize();
        intake.initCV();
        intake.calibrate();

        gp1.initilize(true); // Game Pads can be plugged into the computer
        gp2.initilize(false);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gp1.loop();
            gp2.loop();

            if (gp1.wasLeftBumperPressed()) {
                intake.configureCam(OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }

            if (AA_Operation==Ops.Intake_Manual_Operation){
                intakeManualOperation();
            } else if (AA_Operation==Ops.Test_Intake_Speeds){
                testIntakeSpeeds();
            } else if (AA_Operation==Ops.Test_Intake_Run_To_Position){
                ;
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            dashboard.sendTelemetryPacket(packet);


            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together
            //telemetry.addData("Velocity", 0);
            //telemetry.addData("Encoder", 0);
            //telemetry.addData("Current Velocity", 0);
            //telemetry.addData("Motor Velocity", 0);

            intake.intakeTelemetry();
            telemetry.update();
        }

    }

    public void intakeManualOperation() {
        if (gp1.wasUpPressed()) {
            intake.goToSeek();
        }
        if (gp1.wasDownPressed()) {
            intake.goToGrab();
        }
        if (gp1.wasLeftPressed()) {
            intake.goToUnload();
        }
        if (gp1.wasRightPressed()) {
            intake.flipAndRotateToSampleAndGrab();
        }
        if (gp1.gamepad.left_stick_x < -.25) {
            intake.wrist.setPosition(intake.wrist.getPosition()-.01);
            teamUtil.pause(100);
        } else if (gp1.gamepad.left_stick_x > .25) {
            intake.wrist.setPosition(intake.wrist.getPosition()+.01);
            teamUtil.pause(100);
        }
        if (gp1.wasBPressed()) {
            intake.grab();
        }
        if (gp1.wasYPressed()) {
            intake.release();
        }
        if (gp1.wasXPressed()) {
            intake.grabberReady();
        }
        if(gp1.wasRightBumperPressed()){
            intake.goToSampleAndGrab();
        }
    }

    public void testIntakeSpeeds() {
        if (gp1.gamepad.left_stick_y < -.25) {
            intake.extender.setVelocity(Intake.EXTENDER_MAX_VELOCITY);
        } else if (gp1.gamepad.left_stick_y > .25) {
            intake.extender.setVelocity(-Intake.EXTENDER_MAX_VELOCITY);
        } else {
            intake.extender.setVelocity(0);
        }

        intake.axonSlider.loop();
        if (gp1.gamepad.left_stick_x < -.25) {
            intake.axonSlider.setAdjustedPower(Intake.SLIDER_MAX_VELOCITY);
        } else if (gp1.gamepad.left_stick_x > .25) {
            intake.axonSlider.setAdjustedPower(-Intake.SLIDER_MAX_VELOCITY);
        } else {
            intake.axonSlider.setPower(0);
        }

        if (gp1.wasYPressed()) {
            intake.axonSlider.runToPosition(Intake.SLIDER_UNLOAD);
        }
        if (gp1.wasAPressed()) {
            intake.axonSlider.calibrate(-0.3f, 1);
        }
        if (gp1.wasXPressed()){
            intake.calibrate();
        }
    }
}
