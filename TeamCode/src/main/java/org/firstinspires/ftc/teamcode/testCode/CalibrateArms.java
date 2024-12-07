package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assemblies.AxonSlider;
import org.firstinspires.ftc.teamcode.assemblies.Hang;
import org.firstinspires.ftc.teamcode.assemblies.Intake;
import org.firstinspires.ftc.teamcode.assemblies.Output;
import org.firstinspires.ftc.teamcode.assemblies.Outtake;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
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
    AxonSlider axonSlider;
    boolean hangCalibrated = false;


    public enum Ops {Intake_Manual_Operation,
        Test_Intake_Speeds,
        Test_Intake_Run_To_Position,
        Outtake_Manual_Operation,
        Output_Manual_Operation,
        Hang_Manual_Operation,
        Intake_Fine_Manual_Operation,
        Intake_Seek_Testing
        };
    public static Ops AA_Operation = Ops.Intake_Manual_Operation;
    public static boolean useCV = true;

    public static double SLIDER_TARGET = 0;

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
            case Intake_Fine_Manual_Operation: intakeFineManualOperation();break;
            case Intake_Seek_Testing: intakeSeekTesting();break;

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard



        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.SIDE=teamUtil.Side.BASKET;
        Robot robot = new Robot();
        teamUtil.robot = robot;

        intake = new Intake();
        intake.initialize();
        if (useCV) {
            intake.initCV(true);
            intake.startCVPipeline();
        }

        teamUtil.robot.intake = intake;
        //intake.calibrate();

        outtake = new Outtake();
        outtake.initalize();
        outtake.outakeTelemetry();
        teamUtil.robot.outtake = outtake;


        output = new Output();
        output.initalize();
        //output.calibrate();

        axonSlider = new AxonSlider();

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
            } else if (AA_Operation==Ops.Intake_Fine_Manual_Operation) {
                intakeFineManualOperation();
            }
            else if (AA_Operation==Ops.Intake_Seek_Testing) {
                intakeSeekTesting();
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
            intake.axonSlider.loop();
            intake.intakeTelemetry();
            hang.outputTelemetry();
            telemetry.update();
        }

    }

    public void intakeFineManualOperation() {
        if (gp1.wasRightBumperPressed()) {
            intake.calibrate();
            intake.extender.setVelocity(0);
        }
        if (gp1.wasUpPressed()) {
                intake.goToSampleV2(5000);
        }
        if(gp1.wasLeftPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_SEEK);
        }
        if(gp1.wasRightPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_GRAB);
        }
        if(gp1.wasDownPressed()){
            intake.rotateToSample(intake.sampleDetector.rectAngle.get());
        }
        if(gp1.wasYPressed()){
            intake.grab();
        }
        if(gp1.wasAPressed()){
            intake.grabberReady();
        }
        if(gp1.wasXPressed()){
            intake.grabber.setPosition(Intake.GRABBER_READY);
        }
        if(gp1.wasBPressed()){
            intake.sweeper.setPosition(Intake.SWEEPER_HORIZONTAL_READY);
        }
        if(gp1.wasRightTriggerPressed()){
            intake.wrist.setPosition(Intake.WRIST_MIDDLE);
        }
        if(gp1.wasLeftTriggerPressed()){
            intake.axonSlider.runToPosition(SLIDER_TARGET, 3000);
        }
    }

    public void intakeSeekTesting() {
        if (gp1.wasUpPressed()) {
            intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);

            intake.goToSampleV4(5000,5000);
            //intake.goToSampleAndGrabV2(5000);
        } if (gp1.wasOptionsPressed()) {
            intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);

            intake.goToSampleAndGrabV2(5000);
            //intake.goToSampleAndGrabV2(5000);
        } if(gp1.wasLeftPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_SEEK);
        }if(gp1.wasRightPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_GRAB);
        }if(gp1.wasDownPressed()){
            intake.rotateToSample(intake.sampleDetector.rectAngle.get());
        } if(gp1.wasHomePressed()){
            intake.calibrate();
        }
        if(gp1.wasYPressed()){
            intake.grab();
        }
        if(gp1.wasAPressed()){
            intake.grabberReady();
        }
        if(gp1.wasXPressed()){
            intake.grabber.setPosition(Intake.GRABBER_READY);
        }
        if(gp1.wasBPressed()){
            intake.sweeper.setPosition(Intake.SWEEPER_HORIZONTAL_READY);
        }
        if(gp1.wasRightTriggerPressed()){
            intake.wrist.setPosition(Intake.WRIST_MIDDLE);
        }
    }

    public void intakeManualOperation() {
        if (gp1.wasUpPressed()) {
            intake.goToAndGrabAndUnloadNoWait(7000);
            //intake.flipperGoToSeek(2000);
        }
        if (gp1.wasLeftTriggerPressed()) {
            intake.goToSampleAndGrabNoWaitV2(Intake.GO_TO_SAMPLE_AND_GRAB_NO_WAIT_TIMEOUT);
            //intake.flipperGoToSeek(2000);
        }

        if (gp1.wasDownPressed()) {
            //intake.goToGrab();
            intake.flipperGoToGrab(2000);

        }
        if (gp1.wasLeftPressed()) {
            //intake.goToUnload(2000);
            //intake.flipperGoToUnload(2000);
            intake.unload();
        }

        if (gp1.wasRightPressed()) {
            intake.flipAndRotateToSampleAndGrab(1500);
            //intake.flipperGoToSafe(2000);
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
        if(gp1.wasRightTriggerPressed()){
            intake.calibrate();
        }
//        if(gp1.wasLeftTriggerPressed()){
//            intake.axonSlider.runToPosition(AxonSlider.RIGHT_LIMIT, 6000);
//            intake.axonSlider.setPower(1);
//            teamUtil.pause(500);
//            intake.axonSlider.setPower(0);
//            teamUtil.log("slider potentiometer: " + axonSlider.getPosition());
//            teamUtil.pause(2000);
//            teamUtil.log("slider potentiometer: " + axonSlider.getPosition());
//        }

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
            //intake.axonSlider.runToPosition(AxonSlider.SLIDER_UNLOAD, 1500);
            intake.goToSafeRetractNoWait(4000);
        }
        if (gp1.wasBPressed()) {
            //intake.axonSlider.runToPosition(AxonSlider.SLIDER_READY, 1500);
            intake.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.extender.setVelocity(Intake.EXTENDER_TEST_VELOCITY);
            intake.extender.setTargetPosition(Intake.TEST_EXTENDER_VAL);
            intake.axonSlider.runToPosition(Intake.TEST_SLIDER_VAL, 1500);
            while(intake.extender.isBusy()){
                ;
            }
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
            output.outputHighBucketV2();
        }
        if(gp1.wasBPressed()){
            output.outputHighBucket();
        }
        if(gp1.wasAPressed()){
            output.outputLowBucket();
        }
        if(gp1.wasRightTriggerPressed()){
            output.calibrate();
        }
    }
    public void hangManualOperation(){
        if(gp1.wasXPressed()){
            hang.calibrate();
            hangCalibrated = true;
        }
        if(gp1.wasYPressed() && hangCalibrated){
            hang.extendHang();
        }
        if(gp1.wasBPressed() && hangCalibrated){
            hang.engageHang();
        }

        if(gp1.wasUpPressed()){
            hang.stowHang();
        }
    }
}
