package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.w8wjb.ftc.AdafruitNeoDriver;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.AdafruitNeoDriverImpl3;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Blinkin blinkin;

    public Servo flipper;
    public Servo wrist;
    public Servo sweeper;
    public Servo grabber;
    public DcMotorEx extender;
    public AxonSlider axonSlider = new AxonSlider();
    public AnalogInput flipperPotentiometer;
    public AnalogInput grabberPotentiometer;
    public AnalogInput sweeperPotentiometer;
    AdafruitNeoDriver neopixels;

    public OpenCVSampleDetector sampleDetector = new OpenCVSampleDetector();

    public AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public AtomicBoolean FlipperInUnload = new AtomicBoolean(false);
    public AtomicBoolean FlipperInSeek = new AtomicBoolean(true);
    public AtomicBoolean autoSeeking = new AtomicBoolean(false);

    public static final int NUM_PIXELS = 12;
    public static final int BYTES_PER_PIXEL=4; // RGBW neo pixel device



    static public int SLIDER_MM_DEADBAND = 5;
    static public float SLIDER_MAX_VELOCITY = 0.5f;
    static public float SLIDER_MIN_VELOCITY = 0.05f;
    static public float SLIDER_P_COEFFICIENT = .001f;

    static public int WHITE_NEOPIXEL = 255;
    static public int RED_NEOPIXEL = 0;
    static public int GREEN_NEOPIXEL = 0;
    static public int BLUE_NEOPIXEL = 0;



    //static public float SLIDER_UNLOAD = 300f; // TODO Recalibrate
    //static public float SLIDER_READY = 330f;//TODO Recalibrate

    static public float FLIPPER_SEEK = 0.38f;
    static public double FLIPPER_SEEK_POT_VOLTAGE = 2.008;
    static public double FLIPPER_SEEK_POT_THRESHOLD = .1;
    static public float FLIPPER_UNLOAD = 0.89f;
    static public double FLIPPER_UNLOAD_POT_VOLTAGE = 0.488;
    static public double FLIPPER_UNLOAD_POT_THRESHOLD = 0.1;
    static public float FLIPPER_GRAB = 0.21f;
    static public double FLIPPER_GRAB_POT_VOLTAGE = 2.485;
    static public double FLIPPER_GRAB_POT_THRESHOLD = .01;
    static public float FLIPPER_SAFE = .7f;
    static public double FLIPPER_SAFE_POT_VOLTAGE = 1.045;
    static public double FLIPPER_SAFE_POT_THRESHOLD = .1;
    static public int FLIPPER_GRAB_PAUSE = 500;
    static public long FLIPPER_GO_TO_SEEK_TIMEOUT = 2000;


//    static public float FLIPPER_POTENTIOMETER_SAFE = ;
//    static public float FLIPPER_POTENTIOMETER_SAFE = ;
//    static public float FLIPPER_POTENTIOMETER_SAFE = ;
//    static public float FLIPPER_POTENTIOMETER_SAFE = ;




    static public int FLIPPER_INTO_POS_PAUSE = 1000;

    static public float WRIST_LOAD = 0.5f;
    static public float WRIST_UNLOAD = 0.5f; //0 angle
    static public float WRIST_MIN = 0.17f; // 0 angle
    static public float WRIST_MAX = 0.84f; //179.99 angle
    static public float WRIST_MIDDLE = 0.5f;
    static public int ROTATE_PAUSE = 250;

    /* Values without potentiometer */
    static public float SWEEPER_HORIZONTAL_READY = 0.35f;
    static public float SWEEPER_EXPAND = 0.59f;
    static public float SWEEPER_GRAB = 0.53f;
    static public float SWEEPER_RELEASE = .9f;
    static public float SWEEPER_VERTICAL_READY = 0.5f;

    /* Values with potentiometer--WAITING TO HEAR FROM AXON ON THIS
    static public float SWEEPER_HORIZONTAL_READY = 0.5f; //No Pot .25f
    static public float SWEEPER_HORIZONTAL_READY_POT_VOLTAGE = 1.6535f;
    static public float SWEEPER_VERTICAL_READY = 0.64f;//No Pot .25f
    static public float SWEEPER_VERTICAL_READY_POT_VOLTAGE = 1.295f;
    static public float SWEEPER_GRAB = 0.66f;// No Pot .25f
    static public float SWEEPER_GRAB_POT_VOLTAGE = 1.2415f;
    static public float SWEEPER_EXPAND = 0.7f; //No Pot .25f
    static public float SWEEPER_EXPAND_POT_VOLTAGE = 1.1415f;
    static public float SWEEPER_RELEASE = .93f; //No Pot .25f
    static public float SWEEPER_RELEASE_POT_VOLTAGE = .5747f;
    */
    static public int SWEEPER_POS_COF =1;

    /* Values without potentiometer */
    static public float GRABBER_READY = 0.25f; //No Pot .25f
    static public float GRABBER_GRAB = 0.64f; // No Pot .64f
    static public float GRABBER_RELEASE = .63f; // No Pot .63f TODO: Is this really the right value? Almost the same as grab?

    /* Values with potentiometer--NOT CALIBRATED YET!
    static public float GRABBER_READY = 0.25f; //No Pot .25f
    static public float GRABBER_READY_POT_VOLTAGE = 0;
    static public float GRABBER_GRAB = 0.64f; // No Pot .64f
    static public float GRABBER_GRAB_POT_VOLTAGE = 0;
    static public float GRABBER_RELEASE = .63f; // No Pot .63f TODO: Is this really the right value? Almost the same as grab?
    static public float GRABBER_RELEASE_POT_VOLTAGE = 0;
    */
    static public int GRAB_PAUSE = 500;
    static public int GRAB_DELAY_H = 75;
    static public int GRAB_DELAY2 = 100;
    static public int GRAB_DELAY3 = 75;

    static public float MM_PER_PIX_Y = 0.5625f;
    static public float MM_PER_PIX_X = 0.59375f;
    static public float MM_PER_EXTENDER_TIC = 0.3168f;
    static public float MM_PER_SLIDER_DEGREE = 0.33333f;

    static public float PIX_PER_MM_Y = 1/MM_PER_PIX_Y;
    static public float PIX_PER_MM_X = 1/MM_PER_PIX_X;
    static public float EXTENDER_TIC_PER_MM = 1/MM_PER_EXTENDER_TIC;

    static public int EXTENDER_MAX = 1500; //TODO find this number and use it in movement methods
    static public int EXTENDER_MAX_VELOCITY = 700;
    static public int EXTENDER_MAX_RETRACT_VELOCITY = 2000;
    static public int EXTENDER_MIN_VELOCITY = 50;
    static public int EXTENDER_HOLD_RETRACT_VELOCITY = 200;
    static public int EXTENDER_MM_DEADBAND = 5;
    static public int EXTENDER_P_COEFFICIENT = 4;
    static public int EXTENDER_THRESHOLD = 30;
    static public int EXTENDER_UNLOAD = 5;
    static public int EXTENDER_START_SEEK = 300; // TODO Determine this number
    static public int EXTENDER_CRAWL_INCREMENT = 30;
    static public int EXTENDER_FAST_INCREMENT = 100;
    static public int EXTENDER_MIN = 100;
    static public int EXTENDER_TOLERANCE_RETRACT = 15;
    static public int EXTENDER_RETRACT_TIMEOUT = 3000;
    static public int EXTENDER_SAFE_TO_UNLOAD_THRESHOLD = 50;
    static public int EXTENDER_GO_TO_SAMPLE_VELOCITY = 2000;
    static public int EXTENDER_TOLERANCE_SEEK = 5;


    static public int TEST_EXTENDER_VAL = 1000;
    static public int TEST_SLIDER_VAL = 0;
    static public int EXTENDER_TEST_VELOCITY = 500;




    static public int GO_TO_UNLOAD_WAIT_TIME = 0;
    static public int UNLOAD_WAIT_TIME = 0;
    static public int RELEASE_WAIT_TIME = 500;
    static public int GO_TO_SAMPLE_AND_GRAB_NO_WAIT_TIMEOUT = 10000;

    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;
    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);
    public VisionPortal arduPortal;


    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        //blinkin = new Blinkin(hardwareMap,telemetry);
    }

    public void initialize() {
        teamUtil.log("Initializing Intake");

        flipper = hardwareMap.get(Servo.class,"flipper");
        wrist = hardwareMap.get(Servo.class,"wrist");
        sweeper = hardwareMap.get(Servo.class,"sweeper");
        grabber = hardwareMap.get(Servo.class,"grabber");
        axonSlider.init(hardwareMap,"slider","sliderPotentiometer");
        flipperPotentiometer = hardwareMap.analogInput.get("flipperPotentiometer");
        grabberPotentiometer = hardwareMap.analogInput.get("grabberPotentiometer");
        sweeperPotentiometer = hardwareMap.analogInput.get("sweeperPotentiometer");

        neopixels = hardwareMap.get(AdafruitNeoDriver.class, "intakeleds");
        ((AdafruitNeoDriverImpl3)neopixels).setNumberOfPixelsAndBytesPerPixel(NUM_PIXELS, BYTES_PER_PIXEL);

        extender = hardwareMap.get(DcMotorEx.class,"extender");
        teamUtil.log("extender tolerance " + extender.getTargetPositionTolerance());
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
        sampleDetector.configureCam(arduPortal, true, OpenCVSampleDetector.AEPRIORITY, 1, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
        // TODO: Do we need a pause here?
        sampleDetector.configureCam(arduPortal, OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
        stopCVPipeline();
        teamUtil.log("Initializing CV in Intake - Finished");
    }


    // Calibrate slider and extender.
    public void calibrate() {
        boolean details = false;
        teamUtil.log("Calibrating Intake");

        // Get Grabber into safe position
        /*
        flipper.setPosition(FLIPPER_SEEK);
        while(Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_SEEK_POT_VOLTAGE)>FLIPPER_SEEK_POT_THRESHOLD){
            teamUtil.pause(10);
        }
        FlipperInSeek.set(true);
        FlipperInUnload.set(false);

         */
       if(!flipperGoToSeek(2000)){
            //TODO implement a failsafe in case flipper fails
       }

        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();

        // Calibrate the slider and run to center
        axonSlider.calibrate(-.3f,1);
        axonSlider.runToPosition(AxonSlider.SLIDER_UNLOAD, 1500);
        goToSafe();

        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        extender.setPower(-.2);
        int lastExtenderPosition = extender.getCurrentPosition();
        teamUtil.pause(250);
        while (extender.getCurrentPosition() != lastExtenderPosition) {
            lastExtenderPosition = extender.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: Extender: " + extender.getCurrentPosition());
            teamUtil.pause(50);
        }
        extender.setPower(0);
        teamUtil.pause(500); // let it "relax" just a bit
        extender.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPositionTolerance(EXTENDER_TOLERANCE_RETRACT);// make that our zero position
        extender.setTargetPosition(EXTENDER_UNLOAD);
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);

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
        sampleDetector.reset();
        arduPortal.setProcessorEnabled(sampleDetector, true );
    }
    public void stopCVPipeline () {
        arduPortal.setProcessorEnabled(sampleDetector, false );
    }
    public boolean flipperGoToSeek(long timeout){
        teamUtil.log("flipperGoToSeek has Started. Starting Potentiometer Value: " + flipperPotentiometer.getVoltage()+ "Distance: " + Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_SEEK_POT_VOLTAGE));
        long timeoutTime = System.currentTimeMillis() + timeout;
        boolean details = true;
        flipper.setPosition(FLIPPER_SEEK);
        while(Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_SEEK_POT_VOLTAGE)>FLIPPER_SEEK_POT_THRESHOLD&&teamUtil.keepGoing(timeoutTime)){
            if(details)teamUtil.log("Voltage: " + flipperPotentiometer.getVoltage() + "Target Voltage: " + FLIPPER_SEEK_POT_VOLTAGE);
            teamUtil.pause(10);
        }
        if(!teamUtil.keepGoing(timeoutTime)){
            FlipperInSeek.set(false);
            teamUtil.log("flipperGoToSeek has FAILED");
            return false;
        }
        FlipperInSeek.set(true);
        FlipperInUnload.set(false);
        teamUtil.log("flipperGoToSeek has Finished");
        return true;

    }
    public void flipperGoToSeekNoWait(long timeOut){
        if (moving.get()) {
            teamUtil.log("flipperGoToSeekNoWait called while intake is already moving");
            //TODO fix states
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to goToSafeRetract");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    flipperGoToSeek(timeOut);
                }
            });
            thread.start();
        }
    }
    public boolean flipperGoToSafe(long timeout){
        teamUtil.log("flipperGoToSafe has Started. Starting Potentiometer Value: " + flipperPotentiometer.getVoltage()+ "Distance: " + Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_SAFE_POT_VOLTAGE));
        long timeoutTime = System.currentTimeMillis() + timeout;
        boolean details = true;
        flipper.setPosition(FLIPPER_SAFE);
        while(Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_SAFE_POT_VOLTAGE)>FLIPPER_SAFE_POT_THRESHOLD&&teamUtil.keepGoing(timeoutTime)){
            if(details)teamUtil.log("Voltage: " + flipperPotentiometer.getVoltage() + "Target Voltage: " + FLIPPER_SAFE_POT_VOLTAGE);
            teamUtil.pause(10);
        }
        if(!teamUtil.keepGoing(timeoutTime)) {
            teamUtil.log("flipperGoToSeek has FAILED");
            return false;
        }

        teamUtil.log("flipperGoToSafe has Finished");
        return true;

    }
    public boolean flipperGoToGrab(long timeout){
        teamUtil.log("flipperGoToGrab has Started. Starting Potentiometer Value: " + flipperPotentiometer.getVoltage()+ "Distance: " + Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_GRAB_POT_VOLTAGE));
        long timeoutTime = System.currentTimeMillis() + timeout;
        boolean details = true;
        flipper.setPosition(FLIPPER_GRAB);
        while(Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_GRAB_POT_VOLTAGE)>FLIPPER_GRAB_POT_THRESHOLD&&teamUtil.keepGoing(timeoutTime)){
            if(details)teamUtil.log("Voltage: " + flipperPotentiometer.getVoltage() + "Target Voltage: " + FLIPPER_GRAB_POT_VOLTAGE);
            teamUtil.pause(10);
        }
        if(!teamUtil.keepGoing(timeoutTime)) {
            teamUtil.log("flipperGoToGrab has FAILED");
            return false;
        }

        teamUtil.log("flipperGoToGrab has Finished");
        return true;
    }
    public boolean flipperGoToUnload(long timeout){
        teamUtil.log("flipperGoToUnload has Started. Starting Potentiometer Value: " + flipperPotentiometer.getVoltage() + "Distance: " + Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_UNLOAD_POT_VOLTAGE));
        long timeoutTime = System.currentTimeMillis() + timeout;
        boolean details = true;
        flipper.setPosition(FLIPPER_UNLOAD);
        while(Math.abs(flipperPotentiometer.getVoltage()-FLIPPER_UNLOAD_POT_VOLTAGE)>FLIPPER_UNLOAD_POT_THRESHOLD&&teamUtil.keepGoing(timeoutTime)){
            if(details)teamUtil.log("Voltage: " + flipperPotentiometer.getVoltage() + "Target Voltage: " + FLIPPER_UNLOAD_POT_VOLTAGE);
            teamUtil.pause(10);
        }
        if(!teamUtil.keepGoing(timeoutTime)) {
            teamUtil.log("flipperGoToUnload has FAILED");
            FlipperInUnload.set(false);
            return false;
        }

        teamUtil.log("flipperGoToUnload has Finished");
        FlipperInUnload.set(true);
        FlipperInSeek.set(false);
        return true;

    }


    // Go to seek position
    // Centers first then goes forward a bit
    public void goToSeek(long timeOut){
        teamUtil.log("goToSeek");
        moving.set(true);
        timedOut.set(false);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        flipperGoToSeek(FLIPPER_GO_TO_SEEK_TIMEOUT);
        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();
        axonSlider.runToPosition(axonSlider.SLIDER_READY, timeOut);

        if (axonSlider.timedOut.get()) {
            timedOut.set(true);
            moving.set(false);
            return;
        }
        extendersToPosition(EXTENDER_START_SEEK, timeoutTime-System.currentTimeMillis());
        moving.set(false);
        teamUtil.log("goToSeek--Finished");
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
        flipper.setPosition(FLIPPER_SEEK);
        FlipperInSeek.set(true);

        FlipperInUnload.set(false);

        wrist.setPosition(WRIST_MIDDLE);
        grabberReady();
        axonSlider.runToPosition(axonSlider.SLIDER_READY,timeOut);
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
        autoSeeking.set(true);
        teamUtil.log("Launched GoToSample and Grab" );
        timedOut.set(false);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if(goToSampleV2(timeOut) && !timedOut.get()) {
            flipAndRotateToSampleAndGrab(timeoutTime - System.currentTimeMillis());
            if (!timedOut.get()) {
                goToSafeRetract(timeoutTime - System.currentTimeMillis());

                extendersToPosition(EXTENDER_UNLOAD,timeoutTime-System.currentTimeMillis());
                autoSeeking.set(false);
                return true;
            }
        }
        teamUtil.log("Failed to locate and grab sample" );
        autoSeeking.set(false);
        return false;
    }

    public boolean goToSampleAndGrabV2(long timeOut){
        autoSeeking.set(true);
        teamUtil.log("Launched GoToSample and Grab" );
        timedOut.set(false);
        long timeoutTime = System.currentTimeMillis()+timeOut;
        if(goToSampleV3(timeOut,5000) && !timedOut.get()) {
            flipAndRotateToSampleAndGrab(timeoutTime - System.currentTimeMillis());
            if (!timedOut.get()) {
                goToSafeRetract(timeoutTime - System.currentTimeMillis());

                extendersToPosition(EXTENDER_UNLOAD,timeoutTime-System.currentTimeMillis());
                autoSeeking.set(false);
                return true;
            }
        }
        teamUtil.log("Failed to locate and grab sample" );
        autoSeeking.set(false);
        return false;
    }


    public boolean goToSampleV2(long timeOut){
        teamUtil.log("GoToSample V2 has started");
        long timeoutTime = System.currentTimeMillis() + timeOut;
        boolean details = true;

        startCVPipeline();
        lightsOnandOff(WHITE_NEOPIXEL,RED_NEOPIXEL,GREEN_NEOPIXEL,BLUE_NEOPIXEL,true);
        /*
        flipper.setPosition(FLIPPER_SEEK);
        FlipperInSeek.set(true);
        FlipperInUnload.set(false);

         */
        flipperGoToSeekNoWait(2000);

        grabber.setPosition(GRABBER_READY);
        sweeper.setPosition(SWEEPER_HORIZONTAL_READY);
        wrist.setPosition(WRIST_MIDDLE);
        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double extenderVelocity;
        float sliderVelocity;
        extender.setVelocity(EXTENDER_MAX_VELOCITY);//Tune increment
        if(OpenCVSampleDetector.targetColor== OpenCVSampleDetector.TargetColor.BLUE){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_1);
        }
        else if(OpenCVSampleDetector.targetColor== OpenCVSampleDetector.TargetColor.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED);
        }
        else{
            teamUtil.theBlinkin.setSignal((Blinkin.Signals.YELLOW));
        }
        while(!sampleDetector.foundOne.get()&&extender.getCurrentPosition()<EXTENDER_MAX-10){ // TODO: Need to check for timeout here
            teamUtil.pause(30);
        }


        if(!sampleDetector.foundOne.get()){
            teamUtil.log("Found One False after Search");
            extender.setVelocity(0);
            moving.set(false);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            stopCVPipeline();
            lightsOnandOff(0,0,0,0,false);
            return false;
        }
        else{
            teamUtil.log("Found One True Adjusting X Y LOOP");
            double mmFromCenterX = sampleDetector.rectCenterXOffset.get()*MM_PER_PIX_X;
            double mmFromCenterY = sampleDetector.rectCenterYOffset.get()*MM_PER_PIX_Y;
            while((Math.abs(mmFromCenterY)>EXTENDER_MM_DEADBAND||Math.abs(mmFromCenterX)>SLIDER_MM_DEADBAND)&&teamUtil.keepGoing(timeoutTime)){ // TODO: Need to check for timeout here
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
                    moving.set(false);
                    teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                    stopCVPipeline();
                    lightsOnandOff(0,0,0,0,false);
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
            if(!teamUtil.keepGoing(timeoutTime)){
                teamUtil.log("GoToSample has Timed Out");
                moving.set(false);

                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                stopCVPipeline();
                lightsOnandOff(0,0,0,0,false);
                return false;
            }

        }
        stopCVPipeline();
        moving.set(false);

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
        teamUtil.log("GoToSample has finished--At Block");
        lightsOnandOff(0,0,0,0,false);
        return true;
    }


    public double yPixelsToTicsInZone(double pixels){
        //73 tics for 43.333 pixels
        //51 pixels per inch
        //TODO Implement
        return pixels*(73f/43.3333);
    }

    public double xPixelsToDegreesInZone(double pixels){
        //TODO Implement
        //65 degrees for 51 pixels
        return pixels*(65f/47f);
    }



    public boolean goToSampleV3(long timeOut, long sliderTimeout){
        teamUtil.log("GoToSample V2 has started");
        long timeoutTime = System.currentTimeMillis() + timeOut;
        boolean details = true;

        sampleDetector.reset();
        startCVPipeline();

        //TODO TAKE OUT
        lightsOnandOff(WHITE_NEOPIXEL,RED_NEOPIXEL,GREEN_NEOPIXEL,BLUE_NEOPIXEL,true);
        teamUtil.pause(100);

        flipper.setPosition(FLIPPER_SEEK);
        FlipperInSeek.set(true);
        FlipperInUnload.set(false);

        grabber.setPosition(GRABBER_READY);
        sweeper.setPosition(SWEEPER_HORIZONTAL_READY);
        wrist.setPosition(WRIST_MIDDLE);
        extender.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double extenderVelocity;
        float sliderVelocity;
        extender.setTargetPositionTolerance(EXTENDER_TOLERANCE_SEEK);
        extender.setVelocity(EXTENDER_MAX_VELOCITY);//Tune increment
        if(OpenCVSampleDetector.targetColor== OpenCVSampleDetector.TargetColor.BLUE){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_1);
        }
        else if(OpenCVSampleDetector.targetColor== OpenCVSampleDetector.TargetColor.RED){
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED);
        }
        else{
            teamUtil.theBlinkin.setSignal((Blinkin.Signals.YELLOW));
        }
        /*
        while(!sampleDetector.foundOne.get()&&extender.getCurrentPosition()<EXTENDER_MAX-10) { // TODO: Need to check for timeout here
            teamUtil.pause(30);
        }

         */


        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        double blockX;
        double blockY;

        if(!sampleDetector.foundOne.get()){
            teamUtil.log("Found One False after Search");
            extender.setVelocity(0);
            moving.set(false);
            teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
            //stopCVPipeline(); TODO Put back in
            return false;
        }



        else{

            //stopCVPipeline(); TODO pUt back in
            teamUtil.log("Found One True");
            if(Math.abs(sampleDetector.rectCenterXOffset.get())<154&&Math.abs(sampleDetector.rectCenterYOffset.get())<130){
                blockX = sampleDetector.rectCenterXOffset.get();
                blockY = sampleDetector.rectCenterYOffset.get();
                teamUtil.log("In GoldiLocks Zone");

                double ticsFromCenterY = yPixelsToTicsInZone(blockY);
                double degreesFromCenterX = xPixelsToDegreesInZone(blockX);

                double xPos = axonSlider.getPosition() + degreesFromCenterX;
                if (xPos>AxonSlider.RIGHT_LIMIT|| xPos<AxonSlider.LEFT_LIMIT){
                    teamUtil.log("Required Slider Position Outside of Range");
                    return false;
                }

                double yPos = extender.getCurrentPosition()+ ticsFromCenterY;
                if (yPos<Intake.EXTENDER_MIN|| yPos>Intake.EXTENDER_MAX){
                    teamUtil.log("Required Extender Position Outside of Range");
                    return false;
                }
                extender.setVelocity(EXTENDER_GO_TO_SAMPLE_VELOCITY);

                teamUtil.log("Starting XPos :  " + axonSlider.getPosition() );
                teamUtil.log("Starting YPos :  " + extender.getCurrentPosition());
                teamUtil.log("Target XPos :  " + xPos);
                teamUtil.log("Target YPos :  " + yPos);

                extender.setTargetPosition((int)yPos);
                rotateToSample(sampleDetector.rectAngle.get());
                //TODO THERE IS A BUG!!!! IT SOMETIMES DOESN"T REACH ITS ROTATION PRIOR TO FLIPPING DOWN
                axonSlider.runToPosition(xPos, sliderTimeout);





                axonSlider.setPower(0);
            }
            else{
                /*
                while(Math.abs(sampleDetector.rectCenterXOffset.get())>154&&Math.abs(sampleDetector.rectCenterYOffset.get())>130){
                    if(sampleDetector.rectCenterXOffset.get()>0){
                        axonSlider.setPower(0.5);
                    }else{
                        axonSlider.setPower(-.5);
                    }
                    if(sampleDetector.rectCenterYOffset.get()>0){
                        extender.setPower(1);
                    }else{
                        extender.setPower(-1);
                    }
                }

                axonSlider.setPower(0);
                extender.setVelocity(0);
                blockX = sampleDetector.rectCenterXOffset.get();
                blockY = sampleDetector.rectCenterYOffset.get();

                 */
            }


            /*
            if(!teamUtil.keepGoing(timeoutTime)){
                teamUtil.log("GoToSample has Timed Out");
                moving.set(false);

                teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
                stopCVPipeline();
                return false;
            }

             */

        }
        moving.set(false);

        teamUtil.theBlinkin.setSignal(Blinkin.Signals.DARK_GREEN);
        teamUtil.log("GoToSample has finished--At Block");
        return true;
    }



    public void goToSampleAndGrabNoWait(long timeOut) {
        if (autoSeeking.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToSampleAndGrab while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            autoSeeking.set(true);
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

    public void goToSampleAndGrabNoWaitV2(long timeOut) {
        if (autoSeeking.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to goToSampleAndGrab while intake is moving--ignored");
            return;
        } else {
            moving.set(true);
            autoSeeking.set(true);
            teamUtil.log("Launching Thread to goToSampleAndGrab");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    goToSampleAndGrabV2(timeOut);
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
        flipper.setPosition(FLIPPER_SEEK);
        FlipperInSeek.set(true);

        FlipperInUnload.set(false);

        wrist.setPosition(WRIST_MIDDLE);
        axonSlider.runToPosition(axonSlider.SLIDER_UNLOAD, timeOut);
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

    public void extenderSafeRetractNoWait(long timeOut) {
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
                    extendersToPosition(EXTENDER_UNLOAD,3000);
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
        axonSlider.runToPosition(axonSlider.SLIDER_UNLOAD, timeOut);
        if (axonSlider.timedOut.get()) {
            timedOut.set(true);
            return;
        }
        flipper.setPosition(FLIPPER_UNLOAD);
        FlipperInSeek.set(false);

        FlipperInUnload.set(true);
        wrist.setPosition(WRIST_UNLOAD);
        extendersToPosition(EXTENDER_UNLOAD,timeoutTime-System.currentTimeMillis());
        extender.setVelocity(EXTENDER_HOLD_RETRACT_VELOCITY);
        teamUtil.pause(GO_TO_UNLOAD_WAIT_TIME);
        release();
        teamUtil.pause(RELEASE_WAIT_TIME);
        goToSafe();
        moving.set(false);
        teamUtil.log("goToUnload--Finished");
    }
    public void unload(){
//        flipper.setPosition(FLIPPER_UNLOAD);
//        FlipperInSeek.set(false);
//        FlipperInUnload.set(true);
        flipperGoToUnload(1000);
        wrist.setPosition(WRIST_UNLOAD);
        teamUtil.pause(UNLOAD_WAIT_TIME);
        release();
        teamUtil.pause(RELEASE_WAIT_TIME);
        goToSafe();
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
                    unload();
                    moving.set(false);

                }
            });
            thread.start();
        }
    }


    public void goToSafe(){
        flipper.setPosition(FLIPPER_SAFE);
        FlipperInSeek.set(false);

        FlipperInUnload.set(false);
        wrist.setPosition(WRIST_MIDDLE);
        grabber.setPosition(GRABBER_GRAB);
        sweeper.setPosition(SWEEPER_GRAB);
        //TODO could use flipperGoToSafe insteadd
    }

    public void grabberReady() {
        sweeper.setPosition(SWEEPER_HORIZONTAL_READY);
        grabber.setPosition(GRABBER_READY);
    }
    public void grab(){
        int GRAB_DELAY_1 = (int)((SWEEPER_VERTICAL_READY-sweeper.getPosition())/(SWEEPER_VERTICAL_READY-SWEEPER_HORIZONTAL_READY)*(float) GRAB_DELAY_H);
        sweeper.setPosition(SWEEPER_EXPAND);
        teamUtil.pause(GRAB_DELAY_1);
        grabber.setPosition(GRABBER_GRAB);
        teamUtil.pause(GRAB_DELAY2);
        sweeper.setPosition(SWEEPER_GRAB);
        teamUtil.pause(GRAB_DELAY3);

    }
    public void release() {
        teamUtil.log("Release Called ");

        sweeper.setPosition(SWEEPER_RELEASE);
        grabber.setPosition(GRABBER_RELEASE);
    }

    public void flipAndRotateToSampleAndGrab(long timeOut){
        long timeoutTime = System.currentTimeMillis()+timeOut;
        teamUtil.log("flipAndRotateToSampleAndGrab");
        // TODO: Use timeOut
//        rotateToSample(sampleDetector.rectAngle.get());
//        teamUtil.pause(ROTATE_PAUSE);
        flipperGoToGrab(1000);
//        flipper.setPosition(FLIPPER_GRAB);
//        FlipperInSeek.set(false);
//
//        FlipperInUnload.set(false);
//        teamUtil.pause(FLIPPER_GRAB_PAUSE);
        grab();
        if(System.currentTimeMillis()>timeoutTime){
            timedOut.set(true);
            teamUtil.log("flipAndRotateToSampleAndGrab Has Timed Out");
        }
        teamUtil.log("flipAndRotateToSampleAndGrab Has Finished");
    }

    public void rotateToSample(int rotation){
        teamUtil.log("RotateToSample has started");
        double factor = 0.003722;
        if(rotation<0){
        }
        else {
            if(rotation<90){
                sweeper.setPosition((SWEEPER_VERTICAL_READY-SWEEPER_HORIZONTAL_READY)/90*(rotation)+SWEEPER_HORIZONTAL_READY);
                teamUtil.log("Sweeper position set to: " + ((SWEEPER_VERTICAL_READY-SWEEPER_HORIZONTAL_READY)/90f*(float)(rotation)+SWEEPER_HORIZONTAL_READY));
                wrist.setPosition(0.5-(rotation*factor));
                teamUtil.log("Wrist position set to: " + (.5-(rotation*factor)));
                teamUtil.log("Rotation is: " + rotation);
            }
            else{
                rotation -= 180;
                sweeper.setPosition((SWEEPER_VERTICAL_READY-SWEEPER_HORIZONTAL_READY)/90*(-rotation)+SWEEPER_HORIZONTAL_READY);
                teamUtil.log("Sweeper position set to: " + ((SWEEPER_VERTICAL_READY-SWEEPER_HORIZONTAL_READY)/90*(-rotation)+SWEEPER_HORIZONTAL_READY));
                wrist.setPosition(0.5-(rotation*factor));
                teamUtil.log("Wrist position set to: " + (.5-(rotation*factor)));
                teamUtil.log("Rotation is: " + rotation);
            }


        }
        teamUtil.log("RotateToSample has finished");
    }
    public void extendersToPosition(int position, long timeOut){
        teamUtil.log("extendersToPosition Started: ");
        long timeoutTime = System.currentTimeMillis()+timeOut;
        extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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

    public void manualY(double joystickValue){
        if (moving.get()) { // Output system is already moving in a long running operation
            teamUtil.log("WARNING: Attempt to move extender while intake system is moving--ignored");
        } else {
            extender.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            if(Math.abs(joystickValue) < 0.85){
                if(joystickValue<0){
                    teamUtil.log("Extender Manual: " + (EXTENDER_CRAWL_INCREMENT));
                    extender.setVelocity(EXTENDER_MAX_VELOCITY);

                    extender.setTargetPosition((int) (clamp(extender.getCurrentPosition() + EXTENDER_CRAWL_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));
                    teamUtil.log("Clamped Val: " + (clamp(extender.getCurrentPosition() + EXTENDER_CRAWL_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));

                }else{
                    teamUtil.log("Elev Manual: " + (-EXTENDER_CRAWL_INCREMENT));
                    extender.setVelocity(EXTENDER_MAX_VELOCITY);

                    extender.setTargetPosition((int) (clamp(extender.getCurrentPosition() - EXTENDER_CRAWL_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));
                    teamUtil.log("Clamped Val: " + (clamp(extender.getCurrentPosition() - EXTENDER_CRAWL_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));

                }
            }
            else{
                if(joystickValue<0){
                    teamUtil.log("Elev Manual: " + (EXTENDER_FAST_INCREMENT));
                    extender.setVelocity(EXTENDER_MAX_VELOCITY);

                    extender.setTargetPosition((int) (clamp(extender.getCurrentPosition() + EXTENDER_FAST_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));
                    teamUtil.log("Clamped Val: " + (clamp(extender.getCurrentPosition() + EXTENDER_FAST_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));

                }else{
                    teamUtil.log("Elev Manual: " + (-EXTENDER_FAST_INCREMENT));
                    extender.setVelocity(EXTENDER_MAX_VELOCITY);

                    extender.setTargetPosition((int) (clamp(extender.getCurrentPosition() - EXTENDER_FAST_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));
                    teamUtil.log("Clamped Val: " + (clamp(extender.getCurrentPosition() - EXTENDER_FAST_INCREMENT, EXTENDER_MIN, EXTENDER_MAX)));

                }
            }

        }
    }

    public void manualX(double joystick){
        if(FlipperInSeek.get()){
            axonSlider.manualSliderControl(joystick);
        }
    }
    public void lightsOnandOff(int alpha, int red, int green, int blue, boolean on){
        if(on){
            neopixels.fill(Color.argb(alpha,red,green,blue));
            neopixels.show();
        }
        else{
            neopixels.fill(Color.argb(0,0,0,0));
            neopixels.show();
        }
    }

    public void testWiring() {
        //wrist.setPosition(WRIST_LOAD);
        sweeper.setPosition(SWEEPER_HORIZONTAL_READY);
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
        telemetry.addLine("Flipper: " + flipperPotentiometer.getVoltage() + "V Grabber: " + grabberPotentiometer.getVoltage() + "V Sweeper: " + sweeperPotentiometer.getVoltage() + "V");
    }
}