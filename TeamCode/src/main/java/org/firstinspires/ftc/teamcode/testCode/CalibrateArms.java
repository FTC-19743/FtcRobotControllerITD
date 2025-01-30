package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.WHITE_NEOPIXEL;

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
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
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
        Hang_Manual_Operation,
        Hang_Manual_2,
        Intake_Fine_Manual_Operation,
        Intake_Seek_Testing,
        Test_Axon_Slider,
        Jump_To_Test
        };
    public static Ops AA_Operation = Ops.Intake_Manual_Operation;
    public static boolean useCV = true;

    public static int PICK_UP_HOOKS_PAUSE_1 = 450;
    public static int PICK_UP_HOOKS_PAUSE_2 = 300;
    public static int PICK_UP_HOOKS_PAUSE_3 = 250;

    public static int READY_TO_PLACE_HOOKS_PAUSE_1 = 1000;
    public static int READY_TO_PLACE_HOOKS_VELOCITY = 1400;
    public static int PLACE_HOOKS_VELOCITY = 400;

    public static int DEPLOY_HOOKS_PAUSE_1 = 1500;
    public static int DEPLOY_HOOKS_PAUSE_2 = 500;
    public static int DEPLOY_HOOKS_PAUSE_3 = 500;
    public static int DEPLOY_HOOKS_PAUSE_4 = 500;
    public static int DEPLOY_HOOKS_PAUSE_5 = 500;

    public static long FLIPPER_TEST_TIME = 400;




    public static double SLIDER_TARGET = 0;

    public static int HANG_ITERATOR = 100;
    public static int HANG_VELOCITY = 2800;


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
            case Hang_Manual_2: hangManualOperation2();break;
            case Intake_Fine_Manual_Operation: intakeFineManualOperation();break;
            case Intake_Seek_Testing: intakeSeekTesting();break;
            case Test_Axon_Slider: testAxonSlider();break;
            case Jump_To_Test: testJumpTo();break;

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


        hang = new Hang();
        hang.initalize();
        hangCalibrated = false;

        gp1.initilize(true); // Game Pads can be plugged into the computer
        gp2.initilize(false);
        telemetry.addLine("Ready to start");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gp1.loop();
            gp2.loop();

            if (gp1.wasOptionsPressed()) {
                intake.sampleDetector.configureCam(intake.arduPortal,OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }
            if(gp1.wasRightJoystickFlickedUp()){
                intake.lightsOnandOff(WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);
            }
            if(gp1.wasRightJoystickFlickedDown()){
                intake.lightsOnandOff(WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,false);
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
            } else if (AA_Operation==Ops.Hang_Manual_2){
                hangManualOperation2();
            } else if (AA_Operation==Ops.Intake_Fine_Manual_Operation) {
                intakeFineManualOperation();
            }
            else if (AA_Operation==Ops.Intake_Seek_Testing) {
                intakeSeekTesting();
            }else if (AA_Operation==Ops.Test_Axon_Slider){
                testAxonSlider();
            }else if (AA_Operation==Ops.Jump_To_Test){
                testJumpTo();
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
            //intake.axonSlider.loop();

            intake.intakeTelemetry();
            OpenCVSampleDetectorV2.FrameData frame = intake.sampleDetector.frameDataQueue.peek();
            if (frame != null) {
                telemetry.addLine(String.format("CV X,Y offset in MM: %.1f, %.1f" , intake.xPixelsToMM(frame.rectCenterXOffset) , intake.yPixelsToMM(frame.rectCenterYOffset)));
            }
            outtake.outakeTelemetry();
            output.outputTelemetry();
            hang.outputTelemetry();

            telemetry.addLine("Tolerance: " + intake.extender.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION));



            telemetry.update();
        }

    }

    public void intakeFineManualOperation() {
        if (gp1.wasRightBumperPressed()) {
            intake.calibrate();
            intake.extender.setVelocity(0);
        }
        if (gp1.wasUpPressed()) {
                //intake.goToSampleV2(5000);
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
            intake.axonSlider.runToEncoderPosition(SLIDER_TARGET, false, 3000);
        }
    }

    public void intakeSeekTesting() {
        if (gp1.wasUpPressed()) {
            intake.lightsOnandOff(WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);

            intake.goToSampleV5(5000);
            intake.restartCVPipeline();
            //intake.goToSampleAndGrabV2(5000);
        } if (gp1.wasRightBumperPressed()) {
            intake.lightsOnandOff(WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL,true);

            intake.goToSampleAndGrabV3(false);
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
            //intake.grabberReady();
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
            intake.goToSampleAndGrabV3(true);
            //intake.flipperGoToSeek(2000);
        }
        if (gp1.wasLeftTriggerPressed()) {
            intake.goToSampleAndGrabNoWaitV3(true);
            //intake.flipperGoToSeek(2000);
        }

        if (gp1.wasDownPressed()) {
            //intake.goToGrab();
            intake.flipperGoToGrab(2000);

        }
        if (gp1.wasLeftPressed()) {
            //intake.goToUnload(2000);
            //intake.flipperGoToUnload(2000);
            intake.unloadV2(false);
        }

        if (gp1.wasRightPressed()) {
            intake.flipToSampleAndGrab(1500);
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
            //intake.release();
            intake.retractAll(false,5000);
        }
        if (gp1.wasXPressed()) {
            //intake.grabberReady();
            intake.retractAll(true, 5000);
        }
        if (gp1.wasAPressed()) {
            //intake.goToSampleV2(5000);
            intake.flipper.setPosition(Intake.FLIPPER_SEEK);
        }
        if(gp1.wasRightBumperPressed()){
            //intake.goToSampleAndGrab(5000);
            intake.release();
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
            //intake.axonSlider.runToEncoderPosition(AxonSlider.SLIDER_UNLOAD, 1500);
            intake.goToSafeRetractNoWait(4000);
        }
        if (gp1.wasBPressed()) {
            intake.axonSlider.runToEncoderPosition(AxonSlider.SLIDER_READY, false, 1500);
            intake.extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intake.extender.setVelocity(Intake.EXTENDER_TEST_VELOCITY);
            intake.extender.setTargetPosition(Intake.TEST_EXTENDER_VAL);
            intake.axonSlider.runToEncoderPosition(Intake.TEST_SLIDER_VAL, false, 1500);
            while(intake.extender.isBusy()){
                ;
            }
        }
        if(gp1.wasHomePressed()){
            intake.extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intake.extender.setPower(-1);
            long startTime = System.currentTimeMillis();
            double maxVelo=0;
            while(intake.extender.getCurrentPosition()>Intake.EXTENDER_MIN+100){
                if(Math.abs(intake.extender.getVelocity())>maxVelo){
                    maxVelo=intake.extender.getVelocity();

                }
            }
            intake.extender.setPower(0);
            long elapsedTime = System.currentTimeMillis()-startTime;

            teamUtil.log("Max Velocity: " + maxVelo);
            teamUtil.log("Elapsed Time: " + elapsedTime);

        }
        if (gp1.wasAPressed()) {
            intake.axonSlider.calibrateEncoder(-0.3f);
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
        if(gp1.wasBPressed()){
            dropSampleOutBackAndArmGrab(2000);
        }
    }
    public void outputManualOperation(){
        if(gp1.wasUpPressed()){
            teamUtil.log("In OutputManualOperation");
            output.dropSampleOutBack();
        }
        if(gp1.wasLeftPressed()){
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        }
        if(gp1.wasRightPressed()){
            output.bucket.setPosition(Output.BUCKET_READY_TO_DEPLOY);
        }
        if(gp1.wasDownPressed()){
            output.outputLoad(4000);
        }
        if(gp1.wasYPressed()){
            //output.outputHighBucketV2();
        }
        if(gp1.wasBPressed()){
            output.outputHighBucket(3000);
        }
        if(gp1.wasAPressed()){
            output.outputLowBucket(3000);
        }
        if(gp1.wasRightTriggerPressed()){
            output.calibrate();
        }if(gp1.wasHomePressed()){
            output.lift.setPositionPIDFCoefficients(Output.LIFT_P_COEFFICIENT);
        }if(gp1.wasOptionsPressed()){
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_TOP);
        }
    }
    public void hangManualOperation(){
        if(gp1.wasXPressed()){
            hang.calibrate();
            hangCalibrated = true;
        }
        if(gp1.wasYPressed()){
            hang.extendHang();
        }
        if(gp1.wasBPressed()){
            hang.engageHang();
        }
        if(gp1.wasAPressed()){
            hang.stowHang();
        }

        if(gp1.wasUpPressed()){
            hang.deployHookGrabber();
        }
        if(gp1.wasDownPressed()){
            hang.stowHookGrabber();
        }
        if(gp1.wasLeftPressed()){
            hang.readyHookGrabber();
        }
        if(gp1.wasRightPressed()){
            hang.grabHookGrabber();
        }
        if(gp1.wasHomePressed()){
            hang.releaseHookGrabber();
        }
        hang.joystickDrive(gamepad1.left_stick_x, gamepad1.left_stick_y);
     }

    public void hangManualOperation2(){
        if(gp1.wasAPressed()){
            getReadyToHang();
        }
        if(gp1.wasYPressed()){
            readyToPlaceHooks();
        }
        if(gp1.wasBPressed()){
            placeHooks();
        }
        if(gp1.wasLeftBumperPressed()){
            output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM);

        }if(gp1.wasRightBumperPressed()){
            output.bucket.setPosition(Output.BUCKET_RELOAD);

        }
        if(gp2.wasYPressed()){
            hang.extendHang();
        }
        if(gp2.wasBPressed()){
            hang.engageHang();
        }
        if(gp2.wasAPressed()){
            hang.stowHang();
        }
        if(gp2.wasOptionsPressed()){
            output.lift.setVelocity(PLACE_HOOKS_VELOCITY);
            output.lift.setTargetPosition(Output.LIFT_AT_BAR);
        }


        if(gp1.wasUpPressed()){
            hang.deployHookGrabber();
        }
        if(gp1.wasDownPressed()){
            hang.stowHookGrabber();
        }
        if(gp1.wasLeftPressed()){
            hang.readyHookGrabber();
        }
        if(gp1.wasRightPressed()){
            hang.grabHookGrabber();
        }
        if(gp1.wasHomePressed()){
            hang.releaseHookGrabber();
        }
        hang.joystickDriveV2(gamepad1.left_stick_x, gamepad1.left_stick_y);
        if (hang.stringsTensioned) {
            output.bucket.setPosition(Output.BUCKET_RELOAD); // rotate bucket to avoid bars while climbing
        }

    }

    public void testAxonSlider(){
        if(gp1.wasAPressed()){
            intake.axonSlider.calibrateEncoder(-.2f);
        }
        if(gp1.wasYPressed()){
        }
        if(gp1.wasBPressed()){
        }
        if(gp1.wasLeftBumperPressed()){

        }
    }


    public void testJumpTo(){
        if(gp1.wasLeftBumperPressed()){
            intake.calibrate();
        }
        if(gp1.wasXPressed()){
            intake.lightsOnandOff(Intake.WHITE_NEOPIXEL,Intake.RED_NEOPIXEL,Intake.GREEN_NEOPIXEL,Intake.BLUE_NEOPIXEL, true);
            intake.startCVPipeline();
        }
        if (gp1.wasDownPressed()){
            intake.goToSampleV5(5000);
        }
        if (gp1.wasUpPressed()){
            intake.goToSampleAndGrabV3(false);
        }
        if(gp1.wasLeftPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_GRAB);
        }if(gp1.wasRightPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_SEEK);
        }
        if(gp1.wasAPressed()){
            intake.flipToSampleAndGrab(2000);
        }if(gp1.wasYPressed()){
            intake.flipper.setPosition(Intake.FLIPPER_PRE_GRAB);
            teamUtil.pause(FLIPPER_TEST_TIME);
            intake.flipper.setPosition(Intake.FLIPPER_SEEK);


        }
        if(gp1.wasBPressed()){
            intake.restartCVPipeline();
        }


        if (gp1.gamepad.left_stick_y < -.25) {
            intake.extender.setVelocity(Intake.EXTENDER_MAX_VELOCITY);
        } else if (gp1.gamepad.left_stick_y > .25) {
            intake.extender.setVelocity(-Intake.EXTENDER_MAX_VELOCITY);
        } else {
            intake.extender.setVelocity(0);
        }
    }


    public void pickUpHooks(){
        intake.flipper.setPosition(Intake.FLIPPER_SAFE);
        outtake.outakearm.setPosition(Outtake.ARM_ENGAGE);
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY);
        hang.extendHang();


        output.lift.setTargetPosition(Output.LIFT_SAFE_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_1);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_2);
        output.lift.setTargetPosition(Output.LIFT_PICKUP_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_3);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_GRAB);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_2);

    }

    public void readyToPlaceHooks(){
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(READY_TO_PLACE_HOOKS_PAUSE_1);
        output.lift.setVelocity(READY_TO_PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ABOVE_BAR);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_DEPLOY);
    }

    public void getReadyToHang() {
        pickUpHooks();
        readyToPlaceHooks();
    }



    public void placeHooks(){
        output.lift.setVelocity(PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ONTO_BAR);
        while (output.lift.getCurrentPosition() > Output.LIFT_ONTO_BAR+10) { // wait for hooks to be released
            teamUtil.pause(50);
        }
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY); // Run to near bottom
        output.lift.setTargetPosition(Output.LIFT_DOWN+30);
        while (output.lift.getCurrentPosition() > Output.LIFT_DOWN+40) {
            teamUtil.pause(50);
        }
        output.lift.setVelocity(0); // Turn off lift motor at bottom
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM); // rotate bucket to avoid string while tensioning

    }


    public boolean dropSampleOutBackAndArmGrab(long timeout){
        //TODO Implement Timeout
        if(outtake.outakePotentiometer.getVoltage()>Outtake.POTENTIOMETER_BUCKET_SAFE){
            outtake.outakearm.setPosition(Outtake.ARM_BUCKET_SAFE);
        }
        else{
            teamUtil.log("WARNING: attempted to move with outtake in the way");
            return false;
        }
        output.dropSampleOutBack();
        outtake.outtakeGrab();
        return true;
    }
}
