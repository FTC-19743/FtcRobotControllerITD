package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config // Makes Static data members available in Dashboard
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Servo slider;
    public Servo flipper;
    public Servo wrist;
    public Servo sweeper;
    public Servo grabber;
    public DcMotorEx extender;

    OpenCVSampleDetector sampleDetector = new OpenCVSampleDetector();


    public boolean intakeRunning = false;

    static public float SLIDER_UNLOAD = 0.45f;
    static public float SLIDER_FAR_LEFT = 0.24f;
    static public float SLIDER_FAR_RIGHT = 0.69f;
    static public float SLIDER_INCREMENT = 0.001f;



    static public float FLIPPER_READY = 0.43f;
    static public float FLIPPER_UNLOAD = 0.5f;
    static public float FLIPPER_GRAB = 0.156f;



    static public float WRIST_LOAD = 0.5f;
    static public float WRIST_UNLOAD = 0.5f; //0 angle
    static public float WRIST_MIN = 0.17f; // 0 angle
    static public float WRIST_MAX = 0.84f; //179.99 angle
    //.67 = 135 angle
    //.34 = 45 angle


    static public float SWEEPER_READY = 0.335f;
    static public float SWEEPER_EXPAND = 0.59f;
    static public float SWEEPER_GRAB = 0.53f; // was .59f
    static public float GRABBER_READY = 0.35f;
    static public float GRABBER_GRAB = 0.62f;
    static public int GRAB_DELAY1 = 150;
    static public int GRAB_DELAY2 = 100;

    static public float MM_PER_PIX_Y = 0.5625f;
    static public float MM_PER_PIX_X = 0.59375f;
    static public float MM_PER_EXTENDER_TIC = 0.3168f;
    static public float MM_PER_SLIDER_TIC = 473.333f;

    static public float PIX_PER_MM_Y = 1/MM_PER_PIX_Y;
    static public float PIX_PER_MM_X = 1/MM_PER_PIX_X;
    static public float EXTENDER_TIC_PER_MM = 1/MM_PER_EXTENDER_TIC;
    static public float SLIDER_TIC_PER_MM = 1/MM_PER_SLIDER_TIC;

    static public int EXTENDER_MAX = 1500; //TODO find this number and use it in movement methods
    static public int EXTENDER_VELOCITY = 200; //TODO find this number and use it in movement methods






    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize() {
        teamUtil.log("Initializing Intake");slider = hardwareMap.get(Servo.class,"slider");
        flipper = hardwareMap.get(Servo.class,"flipper");
        wrist = hardwareMap.get(Servo.class,"wrist");
        sweeper = hardwareMap.get(Servo.class,"sweeper");
        grabber = hardwareMap.get(Servo.class,"grabber");
        slider = hardwareMap.get(Servo.class,"slider");


        extender = hardwareMap.get(DcMotorEx.class,"extender");
        extender.setDirection(DcMotorEx.Direction.REVERSE);

        teamUtil.log("Intake Initialized");
    }
    public void calibrate() {
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teamUtil.log("Calibrating Intake: Running down slowly");
        extender.setPower(-.1);
        int lastExtenderPosition = extender.getCurrentPosition();
        teamUtil.pause(250);
        while (extender.getCurrentPosition() != lastExtenderPosition) {
            lastExtenderPosition = extender.getCurrentPosition();
            teamUtil.log("Calibrate Intake: Extender: " + extender.getCurrentPosition());

            teamUtil.pause(50);
        }
        extender.setPower(0);
        teamUtil.log("Calibrate Intake: Stopped Motor");
        extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extender.setTargetPosition(extender.getCurrentPosition());
        teamUtil.log("Calibrate Intake Final: Extender: "+extender.getCurrentPosition());
        extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    public void unload(){
        flipper.setPosition(FLIPPER_READY);
        slider.setPosition(SLIDER_UNLOAD);
        teamUtil.pause(1000);
        wrist.setPosition(WRIST_UNLOAD);
        flipper.setPosition(FLIPPER_UNLOAD);
        teamUtil.pause(1000);
        grabber.setPosition(GRABBER_READY);
    }
    //load up after grab
    public void load(){
        flipper.setPosition(FLIPPER_GRAB);

    }
    public void goToSeek(){
        flipper.setPosition(FLIPPER_READY);
        slider.setPosition(SLIDER_UNLOAD);
        wrist.setPosition(WRIST_UNLOAD);
        sweeper.setPosition(SWEEPER_READY);
        grabber.setPosition(GRABBER_READY);
    }

    public void goToGrab() {
        flipper.setPosition(FLIPPER_GRAB);
        teamUtil.pause(1000);
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

    public void flipAndRotateToSample(int rotation){
        teamUtil.log("FlipAndRotate Has Started");
        rotateToSample(rotation);
        teamUtil.pause(1000);
        flipper.setPosition(FLIPPER_GRAB);
        teamUtil.pause(500);
        grab();
        teamUtil.pause(500);
        flipper.setPosition(FLIPPER_UNLOAD);
        teamUtil.log("FlipAndRotate Has Finished");

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
    public void goToSampleAndGrab(){
        goToSample();
        rotateToSample(sampleDetector.rectAngle.get());
        teamUtil.pause(1000);
        flipper.setPosition(FLIPPER_GRAB);
        //TODO reimplement Grab after testing is finsihed
        //flipAndRotateToSample(sampleDetector.rectAngle.get());
    }
    public boolean goToSample(){
        teamUtil.log("GoToSample has started");
        int extenderLastPosition = extender.getCurrentPosition();
        flipper.setPosition(FLIPPER_READY);
        grabber.setPosition(GRABBER_READY);
        sweeper.setPosition(SWEEPER_READY);
        extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        extender.setVelocity(EXTENDER_VELOCITY);//Tune increment
        while(!sampleDetector.foundOne.get()&&extender.getCurrentPosition()<EXTENDER_MAX-10){
            teamUtil.pause(30);
        }

        //extender.setVelocity(0);
        if(!sampleDetector.foundOne.get()){
            teamUtil.log("Didnt Find One");

            return false;
        }






        double lastSlider = SLIDER_UNLOAD;
        //double startExtender = extender.getCurrentPosition();
        double mmFromCenterX=6;
        double mmFromCenterY=6;
        boolean reachedX=false;
        boolean reachedY=false;
        do {
            if(!sampleDetector.foundOne.get()){
                teamUtil.log("Found One False in DO LOOP");            }
            else{
                teamUtil.log("Found One True Adjusting X Y DO LOOP");
                mmFromCenterX = sampleDetector.rectCenterXOffset.get()*MM_PER_PIX_X;
                if (Math.abs(mmFromCenterX)>5&&!reachedX)    {
                    teamUtil.log("MM from Center X"+ mmFromCenterX);

                    if(mmFromCenterX<0){
                        slider.setPosition(lastSlider-SLIDER_INCREMENT);
                        teamUtil.log("Slider Moving Left");
                    }else{
                        slider.setPosition(lastSlider+SLIDER_INCREMENT);
                        teamUtil.log("Slider Moving Right");
                    //TODO For some reason it is only fast in one direction
                    }
                    lastSlider=slider.getPosition();
                }
                else{
                    reachedX = true;
                    slider.setPosition(lastSlider);
                }



                mmFromCenterY = sampleDetector.rectCenterYOffset.get()*MM_PER_PIX_Y;
                if (Math.abs(mmFromCenterY)>5&&!reachedY){

                    teamUtil.log("MM from Center Y"+ mmFromCenterY);

                    if(mmFromCenterY>0){
                        teamUtil.log("Going Forward");
                        extender.setVelocity(200);
                    }else{
                        teamUtil.log("Going Backward");

                        extender.setVelocity(-200);
                    }
                }else{
                    reachedY = true;
                    extender.setVelocity(0);
                }
                teamUtil.log("Reached X: " + reachedX + "Reached Y: " + reachedY);


                //double ticsFromCenterY = mmFromCenterY/MM_PER_EXTENDER_TIC;
                //double ticsFromCenterX = mmFromCenterX/MM_PER_SLIDER_TIC;

                //extender.setTargetPosition((int)startExtender-(int)ticsFromCenterY);
                //slider.setPosition(startSlider-ticsFromCenterX);

                if(lastSlider<SLIDER_FAR_LEFT||lastSlider>SLIDER_FAR_RIGHT){

                    teamUtil.log("Outside Slider Limit");
                    return false;
                }

                if(extender.getCurrentPosition()>EXTENDER_MAX){
                    extender.setVelocity(0);
                    teamUtil.log("Outside Extender Limit");
                    return false;
                }

                teamUtil.pause(30);

            }
        }  while(!reachedX||!reachedY);




        teamUtil.log("At Block");
        teamUtil.log("GoToSample has finished");
        return true;

    }

    public void testWiring() {
        //wrist.setPosition(WRIST_LOAD);
        sweeper.setPosition(SWEEPER_READY);
        //grabber.setPosition(GRABBER_READY);
        //flipper.setPosition(FLIPPER_READY);
        //slider.setPosition(SLIDER_UNLOAD);
    }
    public void intakeTelemetry(){
        telemetry.addLine("Extender Position: " + extender.getCurrentPosition());
    }
}