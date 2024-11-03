package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.assemblies.AxonSlider;
import org.firstinspires.ftc.teamcode.assemblies.Hang;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.assemblies.Outtake;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Calibrate Arms", group = "Test Code")
public class CalibrateArms extends LinearOpMode {

    Intake intake;
    Outtake outtake;
    Output output;
    Hang hang;
    boolean hangCalibrated = false;

    public enum Ops {Intake_Manual_Operation,
        Test_Intake_Speeds,
        Test_Intake_Run_To_Position,
        Outtake_Manual_Operation,
        Output_Manual_Operation,
        Hang_Manual_Operation
        };
    public static Ops AA_Operation = Ops.Intake_Manual_Operation;



    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();

    private void doAction() {
        switch (AA_Operation) {
            case Intake_Manual_Operation : intakeManualOperation();break;
            case Test_Intake_Speeds : testIntakeSpeeds();break;
            case Test_Intake_Run_To_Position : ;break;
            case Outtake_Manual_Operation: outtakeManualOperation();break;
            case Output_Manual_Operation: outputManualOperation();break;
            case Hang_Manual_Operation: hangManualOperation();break;
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
        intake.initCV(true);
        intake.calibrate();

        outtake = new Outtake();
        outtake.initalize();
        outtake.outakeTelemetry();

        output = new Output();
        output.initalize();
        output.calibrate();


        hang = new Hang();
        hang.initalize();
        hangCalibrated = false;

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
                intake.sampleDetector.configureCam(intake.arduPortal,OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }

            if (AA_Operation==Ops.Intake_Manual_Operation){
                intakeManualOperation();
            } else if (AA_Operation==Ops.Test_Intake_Speeds){
                testIntakeSpeeds();
            } else if (AA_Operation==Ops.Test_Intake_Run_To_Position){
                ;
            } else if (AA_Operation==Ops.Outtake_Manual_Operation) {
                outtakeManualOperation();
            } else if (AA_Operation==Ops.Output_Manual_Operation){
                outputManualOperation();
            } else if (AA_Operation==Ops.Hang_Manual_Operation){
                hangManualOperation();
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
            outtake.outakeTelemetry();
            output.outputTelemetry();
            intake.intakeTelemetry();
            hang.outputTelemetry();
            telemetry.update();
        }

    }

    public void intakeManualOperation() {
        if (gp1.wasUpPressed()) {
            intake.goToSeek(1500);
        }
        if (gp1.wasDownPressed()) {
            intake.goToGrab();
        }
        if (gp1.wasLeftPressed()) {
            intake.goToUnload(2000);
        }
        if (gp1.wasRightPressed()) {
            intake.flipAndRotateToSampleAndGrab(1500);
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
        if (gp1.wasAPressed()) {
            intake.goToSampleV2(5000);
        }
        if(gp1.wasRightBumperPressed()){
            intake.goToSampleAndGrab(5000);
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
            intake.axonSlider.runToPosition(AxonSlider.SLIDER_UNLOAD, 1500);
        }
        if (gp1.wasBPressed()) {
            intake.axonSlider.runToPosition(AxonSlider.SLIDER_READY, 1500);
        }
        if (gp1.wasAPressed()) {
            intake.axonSlider.calibrate(-0.3f, 1);
        }
        if (gp1.wasXPressed()){
            intake.calibrate();
        }
    }
    public void outtakeManualOperation(){
        if(gp1.wasUpPressed()){
            outtake.outakearm.setPosition(Outtake.ARM_UP);
        }
        if(gp1.wasDownPressed()){
            outtake.outakearm.setPosition(Outtake.ARM_DOWN);
        }
        if(gp1.wasYPressed()){
            outtake.outakewrist.setPosition(Outtake.WRIST_GRAB);
        }
        if(gp1.wasAPressed()){
            outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
        }
    }
    public void outputManualOperation(){
        if(gp1.wasUpPressed()){
            teamUtil.log("In OutputManualOperation");
            output.dropSampleOutBack();
        }
        if(gp1.wasDownPressed()){
            output.outputLoad(4000);
        }
        if(gp1.wasYPressed()){
            output.outputHighBucket();
        }
        if(gp1.wasAPressed()){
            output.outputLowBucket();
        }
    }
    public void hangManualOperation(){
        if(gp1.wasXPressed()){
            hang.calibrate();
            hangCalibrated = true;
        }
        if(gp1.wasYPressed() && hangCalibrated){
            hang.extendHang(5000);
        }
        if(gp1.wasBPressed() && hangCalibrated){
            hang.engageHang(5000);
        }
        if(gp1.wasAPressed()){
            output.outputLowBucket();
        }
    }
}
