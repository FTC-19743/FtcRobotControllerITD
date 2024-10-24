package org.firstinspires.ftc.teamcode.assemblies;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Servo flipper;
    public Servo wrist;
    public Servo sweeper;
    public Servo grabber;
    public DcMotorEx extender;
    public AxonSlider axonSlider = new AxonSlider();

    public OpenCVSampleDetector sampleDetector = new OpenCVSampleDetector();

    public AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);

    static public float SLIDER_UNLOAD = 300f; // TODO Recalibrate
    static public float SLIDER_READY = 330f;//TODO Recalibrate
    static public int SLIDER_MM_DEADBAND = 5;
    static public float SLIDER_MAX_VELOCITY = 0.5f;
    static public float SLIDER_MIN_VELOCITY = 0.05f;
    static public float SLIDER_P_COEFFICIENT = .001f;

    static public float FLIPPER_READY = 0.43f;
    static public float FLIPPER_UNLOAD = 0f;
    static public float FLIPPER_GRAB = 0.785f;
    static public int FLIPPER_GRAB_PAUSE = 500;
    static public float FLIPPER_SAFE = .24f;

    static public float WRIST_LOAD = 0.5f;
    static public float WRIST_UNLOAD = 0.84f; //0 angle
    static public float WRIST_MIN = 0.17f; // 0 angle
    static public float WRIST_MAX = 0.84f; //179.99 angle
    static public float WRIST_MIDDLE = 0.5f;
    static public int ROTATE_PAUSE = 250;

    static public float SWEEPER_READY = 0.330f;
    static public float SWEEPER_EXPAND = 0.59f;
    static public float SWEEPER_GRAB = 0.53f; // was .59f

    static public float GRABBER_READY = 0.25f;
    static public float GRABBER_GRAB = 0.63f;
    static public int GRAB_PAUSE = 250;
    static public int GRAB_DELAY1 = 150;
    static public int GRAB_DELAY2 = 100;

    static public float MM_PER_PIX_Y = 0.5625f;
    static public float MM_PER_PIX_X = 0.59375f;
    static public float MM_PER_EXTENDER_TIC = 0.3168f;
    static public float MM_PER_SLIDER_DEGREE = 0.33333f;

    static public float PIX_PER_MM_Y = 1/MM_PER_PIX_Y;
    static public float PIX_PER_MM_X = 1/MM_PER_PIX_X;
    static public float EXTENDER_TIC_PER_MM = 1/MM_PER_EXTENDER_TIC;

    static public int EXTENDER_MAX = 1500; //TODO find this number and use it in movement methods
    static public int EXTENDER_MAX_VELOCITY = 700;
    static public int EXTENDER_MAX_RETRACT_VELOCITY = 1500;
    static public int EXTENDER_MIN_VELOCITY = 50;
    static public int EXTENDER_MM_DEADBAND = 5;
    static public int EXTENDER_P_COEFFICIENT = 4;
    static public int EXTENDER_THRESHOLD = 10;
    static public int EXTENDER_UNLOAD = 0;
    static public int EXTENDER_START_SEEK = 100; // TODO Determine this number

    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;
    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);
    public VisionPortal arduPortal;


    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize() {
        teamUtil.log("Initializing Intake");

        flipper = hardwareMap.get(Servo.class,"flipper");
        wrist = hardwareMap.get(Servo.class,"wrist");
        sweeper = hardwareMap.get(Servo.class,"sweeper");
        grabber = hardwareMap.get(Servo.class,"grabber");
        axonSlider.init(hardwareMap,"slider","axonPotentiometer");

        extender = hardwareMap.get(DcMotorEx.class,"extender");
        extender.setDirection(DcMotorEx.Direction.REVERSE);
        extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        teamUtil.log("Intake Initialized");
    }

    public void initCV(boolean enableLiveView){
        teamUtil.log("Initializing CV in Intake");
        CameraName arducam = (CameraName)hardwareMap.get(WebcamName.class, "arducam"); // arducam  logitechhd
        CameraCharacteristics chars = arducam.getCameraCharacteristics();

        VisionPortal.Builder armBuilder = new VisionPortal.Builder();
        armBuilder.setCamera(arducam);
        armBuilder.enableLiveView(enableLiveView);

        // Can also set resolution and stream format if we want to optimize resource usage.
        armBuilder.setCameraResolution(arduSize);
        //armBuilder.setStreamFormat(TBD);

        armBuilder.addProcessor(sampleDetector);
        arduPortal = armBuilder.build();
        sampleDetector.viewingPipeline = enableLiveView;

        // Wait for the camera to be open
        if (arduPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!teamUtil.theOpMode.isStopRequested() && (arduPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                teamUtil.pause(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }
        sampleDetector.configureCam(arduPortal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
        teamUtil.log("Initializing CV in Intake - Finished");
    }


    // Calibrate slider and extender.
    public void calibrate() {
        boolean details = false;
        teamUtil.log("Calibrating Intake");

        // Get Grabber into safe position
        flipper.setPosition(FLIPPER_READY);
        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();

        // Calibrate the slider and run to center
        axonSlider.calibrate(-.3f,1);
        axonSlider.runToPosition(SLIDER_UNLOAD, 1500);
        goToSafe();

        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setPower(-.1);
        int lastExtenderPosition = extender.getCurrentPosition();
        teamUtil.pause(250);
        while (extender.getCurrentPosition() != lastExtenderPosition) {
            lastExtenderPosition = extender.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: Extender: " + extender.getCurrentPosition());
            teamUtil.pause(50);
        }
        extender.setPower(0);
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(extender.getCurrentPosition());
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teamUtil.log("Calibrate Intake Final: Extender: "+extender.getCurrentPosition());
    }

    public void setTargetColor(OpenCVSampleDetector.TargetColor targetColor){
        sampleDetector.setTargetColor(targetColor);
    }

    public boolean currentlyStreaming() {
        return (arduPortal.getCameraState() == VisionPortal.CameraState.STREAMING);
    }

    public void startStreaming(){
        // TODO
    }

    public void stopStreaming () {
        // TODO
    }
    public void startCVPipeline () {
        arduPortal.setProcessorEnabled(sampleDetector, true );
    }
    public void stopCVPipeline () {
        arduPortal.setProcessorEnabled(sampleDetector, false );
    }

    // Go to seek position
    // Centers first then goes forward a bit
    public void goToSeek(long timeOut){
        teamUtil.log("goToSeek");
        moving.set(true);
        timedOut.set(false);
        long timeoutTime = System.currentTimeMillis()+timeOut;

        flipper.setPosition(FLIPPER_READY);
        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();
        axonSlider.runToPosition(SLIDER_READY, timeOut);

        if (axonSlider.timedOut.get()) {
            timedOut.set(true);
            moving.set(false);
            return;
        }
        extendersToPosition(EXTENDER_START_SEEK, timeoutTime-System.currentTimeMillis());
        moving.set(false);
        teamUtil.log("goToUnload--Finished");
    }
    public void goToSeekNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToSeek while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToSeek");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToSeek(timeOut);
                }
            });
            thread.start();
        }
    }

    public void goToSeekGrabberOnly(long timeOut){
        teamUtil.log("goToSeekGrabberOnly");
        moving.set(true);
        timedOut.set(false);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        flipper.setPosition(FLIPPER_READY);
        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();
        axonSlider.runToPosition(SLIDER_READY,timeOut);
        teamUtil.log("goToSeekGrabberOnly--Finished");
    }
    public void goToSeekGrabberOnlyNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToSeekGrabberOnly while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToSeekGrabberOnly");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToSeek(timeOut);
                }
            });
            thread.start();
        }
    }

    // Assumes we are already in a position to start seeking a sample
    // Returns true if it thinks we got one, false if it gave up or timed out
    // Leaves extenders extended and grabber in safe retract position
    public boolean goToSampleAndGrab(long timeOut){
        teamUtil.log("Failed to locate and grab sample" );
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if(goToSampleV2(timeOut) && !timedOut.get()) {
            flipAndRotateToSampleAndGrab(timeoutTime - System.currentTimeMillis());
            if (!timedOut.get()) {
                goToSafeRetract(timeoutTime - System.currentTimeMillis());
                return true;
            }
        }
        teamUtil.log("Failed to locate and grab sample" );
        return false;
    }
    public boolean goToSampleV2(long timeOut){
        teamUtil.log("GoToSample V2 has started");
        boolean details = true;
        flipper.setPosition(FLIPPER_READY);
        grabber.setPosition(GRABBER_READY);
        sweeper.setPosition(SWEEPER_READY);
        wrist.setPosition(WRIST_MIDDLE);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double extenderVelocity;
        float sliderVelocity;
        extender.setVelocity(EXTENDER_MAX_VELOCITY);//Tune increment
        while(!sampleDetector.foundOne.get()&&extender.getCurrentPosition()<EXTENDER_MAX-10){
            teamUtil.pause(30);
        }


        if(!sampleDetector.foundOne.get()){
            teamUtil.log("Found One False after Search");
            extender.setVelocity(0);
        }
        else{
            teamUtil.log("Found One True Adjusting X Y LOOP");
            double mmFromCenterX = sampleDetector.rectCenterXOffset.get()*MM_PER_PIX_X;
            double mmFromCenterY = sampleDetector.rectCenterYOffset.get()*MM_PER_PIX_Y;




            while(Math.abs(mmFromCenterY)>EXTENDER_MM_DEADBAND||Math.abs(mmFromCenterX)>SLIDER_MM_DEADBAND){ //tentative values all needs to be tuned
                axonSlider.loop();
                if(Math.abs(mmFromCenterY)<=EXTENDER_MM_DEADBAND){
                    extenderVelocity=0;
                }else{
                    extenderVelocity = Math.min(EXTENDER_P_COEFFICIENT*Math.abs(mmFromCenterY)+EXTENDER_MIN_VELOCITY,EXTENDER_MAX_VELOCITY);
                    if(mmFromCenterY<0){
                        extenderVelocity*=-1;
                    }
                }

                if(mmFromCenterX>(axonSlider.RIGHT_LIMIT -axonSlider.getPosition())*MM_PER_SLIDER_DEGREE||mmFromCenterX<(axonSlider.LEFT_LIMIT -axonSlider.getPosition())*MM_PER_SLIDER_DEGREE){
                    extender.setVelocity(0);
                    axonSlider.setPower(0);
                    teamUtil.log("Target slider position is beyond mechanical range. Failing out.");
                    return false;
                }

                if(Math.abs(mmFromCenterX)<=SLIDER_MM_DEADBAND){
                    sliderVelocity = 0;
                }else{
                    sliderVelocity = (float) Math.min(SLIDER_P_COEFFICIENT*Math.abs(mmFromCenterX)+SLIDER_MIN_VELOCITY,SLIDER_MAX_VELOCITY);
                    if(mmFromCenterX>0){
                        sliderVelocity*=-1;
                    }
                }
                extender.setVelocity(extenderVelocity);
                axonSlider.setAdjustedPower(sliderVelocity);
                mmFromCenterY = sampleDetector.rectCenterYOffset.get()*MM_PER_PIX_Y;
                mmFromCenterX = sampleDetector.rectCenterXOffset.get()*MM_PER_PIX_X;
                if (details) teamUtil.log("MM from Center X: " + mmFromCenterX + " Y: " + mmFromCenterY);
                if (details) teamUtil.log("slide power: " + sliderVelocity + " extender power: " + extenderVelocity);
                teamUtil.pause(30);

            }
            extender.setVelocity(0);
            axonSlider.setPower(0);
        }

        teamUtil.log("At Block");
        teamUtil.log("GoToSample has finished");
        return true;

    }
    public void goToSampleAndGrabNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToSampleAndGrab while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToSampleAndGrab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToSampleAndGrab(timeOut);
                }
            });
            thread.start();
        }
    }

    // Go to ready position with wrist level
    // Useful for retracting extender without hitting lower bar
    public void goToSafeRetract(long timeOut) {
        teamUtil.log("goToSafeRetract");
        long timeoutTime = System.currentTimeMillis()+timeOut;
        moving.set(true);
        timedOut.set(false);
        flipper.setPosition(FLIPPER_READY);
        wrist.setPosition(WRIST_MIDDLE);
        axonSlider.runToPosition(SLIDER_UNLOAD, timeOut);
        timedOut.set(axonSlider.timedOut.get());
        moving.set(false);
        teamUtil.log("goToSafeRetract--Finished");
    }
    public void goToSafeRetractNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to GoToSafeRetract while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToSafeRetract");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToSafeRetract(timeOut);
                }
            });
            thread.start();
        }

    }

    // Go to unload position
    // Centers first then goes back  TODO: // Maybe could be made a bit faster by pulling extenders back as soon as it is safe
    public void goToUnload(long timeOut) {
        teamUtil.log("goToUnload");
        long timeoutTime = System.currentTimeMillis()+timeOut;
        moving.set(true);
        timedOut.set(false);
        axonSlider.runToPosition(SLIDER_UNLOAD, timeOut);
        if (axonSlider.timedOut.get()) {
            timedOut.set(true);
            return;
        }
        flipper.setPosition(FLIPPER_UNLOAD);
        wrist.setPosition(WRIST_UNLOAD);
        extendersToPosition(EXTENDER_UNLOAD,timeoutTime-System.currentTimeMillis());
        moving.set(false);
        teamUtil.log("goToUnload--Finished");
    }
    public void goToUnloadNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToUnload while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToUnload");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToUnload(timeOut);
                }
            });
            thread.start();
        }
    }
    public void unloadNoWait(long timeOut) {
        if (moving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToUnload while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToUnload");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToUnload(timeOut);
                    release();
                }
            });
            thread.start();
        }
    }


    public void goToSafe(){
        flipper.setPosition(FLIPPER_SAFE);
        wrist.setPosition(WRIST_MIDDLE);
        grabber.setPosition(GRABBER_GRAB);
        sweeper.setPosition(SWEEPER_GRAB);
    }
    public void goToGrab() {
        flipper.setPosition(FLIPPER_GRAB);
        teamUtil.pause(1000);
    }
    public void grabberReady() {
        sweeper.setPosition(SWEEPER_READY);
        grabber.setPosition(GRABBER_READY);
    }
    public void grab(){
        sweeper.setPosition(SWEEPER_EXPAND);
        teamUtil.pause(GRAB_DELAY1);
        grabber.setPosition(GRABBER_GRAB);
        teamUtil.pause(GRAB_DELAY2);
        sweeper.setPosition(SWEEPER_GRAB);
    }
    public void release() {
        sweeper.setPosition(SWEEPER_EXPAND);
        grabber.setPosition(GRABBER_READY);
    }

    public void flipAndRotateToSampleAndGrab(long timeOut){
        teamUtil.log("flipAndRotateToSampleAndGrab");
        // TODO: Use timeOut
        rotateToSample(sampleDetector.rectAngle.get());
        teamUtil.pause(ROTATE_PAUSE);
        flipper.setPosition(FLIPPER_GRAB);
        teamUtil.pause(FLIPPER_GRAB_PAUSE);
        grab();
        teamUtil.pause(GRAB_PAUSE);
        teamUtil.log("flipAndRotateToSampleAndGrab Has Finished");
    }

    public void rotateToSample(int rotation){
        teamUtil.log("RotateToSample has started");
        double factor = 0.003722;
        if(rotation<0){
        }
        else {
            if(rotation<90){
                wrist.setPosition(0.5-(rotation*factor));
                teamUtil.log("Wrist position set to: " + (.5-(rotation*factor)));
            }
            else{
                rotation -= 180;
                wrist.setPosition(0.5-(rotation*factor));
                teamUtil.log("Wrist position set to: " + (.5-(rotation*factor)));
            }
        }
        teamUtil.log("RotateToSample has finished");
    }
    public void extendersToPosition(int position, long timeOut){
        long timeoutTime = System.currentTimeMillis()+timeOut;
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        extender.setTargetPosition(position);
        extender.setVelocity(EXTENDER_MAX_RETRACT_VELOCITY);
        while (teamUtil.keepGoing(timeoutTime) && Math.abs(extender.getCurrentPosition() - position) > EXTENDER_THRESHOLD) {
            teamUtil.pause(50);
        }
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("extendersToPosition TIMED OUT: ");
        } else {
            teamUtil.log("extendersToPosition Finished" );
        }
    }



    public void testWiring() {
        //wrist.setPosition(WRIST_LOAD);
        sweeper.setPosition(SWEEPER_READY);
        //grabber.setPosition(GRABBER_READY);
        //flipper.setPosition(FLIPPER_READY);
        //slider.setPosition(SLIDER_UNLOAD);
    }
    public void intakeTelemetry() {
        telemetry.addLine("Current Color: " + sampleDetector.targetColor);
        telemetry.addLine("Found One: " + sampleDetector.foundOne.get()
                + " Offset: " + sampleDetector.rectCenterXOffset.get() + ", " + sampleDetector.rectCenterYOffset.get()
                + " Angle: " + sampleDetector.rectAngle.get());

        telemetry.addLine("Intake Extender Position: " + extender.getCurrentPosition());
        telemetry.addLine("Axon Slider Position : " + axonSlider.getPosition() + " (0-360): " + (int)axonSlider.getDegrees360() + " Voltage: " + axonSlider.axonPotentiometer.getVoltage());
    }
}