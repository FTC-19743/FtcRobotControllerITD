package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcontroller.external.samples.UtilityOctoQuadConfigMenu;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Output {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx lift;
    public Servo bucket;
    public Intake intake;
    public Outtake outtake;
    public AtomicBoolean outputMoving = new AtomicBoolean(false);
    public AtomicBoolean outputLiftAtBottom = new AtomicBoolean(true);

    public boolean details;

    static public int LIFT_MAX = 1770;
    static public int LIFT_MAX_VELOCITY = 2800;
    static public int LIFT_MAX_POWER = 1;
    static public int LIFT_MIN_VELOCITY = 200;
    static public int LIFT_DOWN = 2;
    static public int LIFT_TOP_BUCKET = 1750;
    static public int LIFT_MIDDLE_BUCKET = 880; // TODO Determine this number
    static public double LIFT_P_COEFFICIENT = 10;

    static public float BUCKET_DEPLOY_AT_BOTTOM = 0.1f;
    static public float BUCKET_DEPLOY_AT_TOP = 0.24f;
    static public float BUCKET_SAFE = 0.66f;
    static public float BUCKET_READY_TO_DEPLOY = 0.35f; //TODO Possibly use for going up to buckets (optimization for driver)
    static public float BUCKET_RELOAD = 0.7f; //was .66
    static public float BUCKET_TRAVEL = 0.4f;

    static public int DROP_SAMPLE_TIME = 1000;
    static public int DROP_SAMPLE_TIME_2 = 1000;
    static public int BUCKET_LOAD_PAUSE = 200;



    public Output() {
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Output");
        lift = hardwareMap.get(DcMotorEx.class,"lift");
        bucket = hardwareMap.get(Servo.class,"bucket");


        teamUtil.log("Intake Output");
    }
    public void calibrate(){
        bucket.setPosition(BUCKET_RELOAD);

        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setPower(-.2);
        int lastExtenderPosition = lift.getCurrentPosition();
        teamUtil.pause(250);
        while (lift.getCurrentPosition() != lastExtenderPosition) {
            lastExtenderPosition = lift.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: Extender: " + lift.getCurrentPosition());
            teamUtil.pause(50);
        }
        lift.setPower(0);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(lift.getCurrentPosition());
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        teamUtil.log("Calibrate Intake Final: Extender: "+lift.getCurrentPosition());
        outputLiftAtBottom.set(true);
    }

    public void outputTelemetry(){
        telemetry.addLine("Output Lift Position: " + lift.getCurrentPosition());
        telemetry.addLine("Output PIDF COEFFICIENTS: " + lift.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION)); //TODO TAKE OUT

    }

    public void dropSampleOutBack(){
        outputMoving.set(true);
        if(outputLiftAtBottom.get()){
            bucket.setPosition(BUCKET_DEPLOY_AT_BOTTOM);

        }else{
            bucket.setPosition(BUCKET_DEPLOY_AT_TOP);

        }
        teamUtil.pause(DROP_SAMPLE_TIME);
        if(outputLiftAtBottom.get()){
            bucket.setPosition(BUCKET_RELOAD);
            teamUtil.pause((DROP_SAMPLE_TIME_2));
        }
        else{
            bucket.setPosition(BUCKET_SAFE);
        }
        outputMoving.set(false);
        teamUtil.log("DropSampleOutBack Completed");
    }


    public void dropSampleOutBackNoWait(){
        if(outputMoving.get()){
            teamUtil.log("WARNING: Attempt to dropSampleOutBackNoWait while output is moving--ignored");
        }
        else{
            outputMoving.set(true);
            teamUtil.log("Launching Thread to dropSampleOutBackNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    dropSampleOutBack();
                }

            });
            thread.start();
        }
    }
    public void outputLoad(long timeout){
        intake = teamUtil.robot.intake;
        outtake = teamUtil.robot.outtake;
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if(intake.FlipperInUnload.get()||outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_OUTPUT_CLEAR){
            teamUtil.log("Couldn't run Output Load Because Stuff is in the way");
        }
        else{
            outputMoving.set(true);
            bucket.setPosition(BUCKET_SAFE);
            teamUtil.pause(BUCKET_LOAD_PAUSE);
            teamUtil.log("Go To Load: Running to Bottom");
            lift.setTargetPosition(LIFT_DOWN);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            long timeOutTime2 = System.currentTimeMillis() + timeout;
            while (teamUtil.keepGoing(timeOutTime2)&&lift.getCurrentPosition() > LIFT_DOWN+10) {
                teamUtil.pause(50);
            }
            lift.setVelocity(0);
            bucket.setPosition(BUCKET_RELOAD);

            outputLiftAtBottom.set(true);
        }
        outputMoving.set(false);


    }

    public void outputLoadNoWait(long timeout){
        if(outputMoving.get()){
            teamUtil.log("WARNING: Attempt to outputLoad while output is moving--ignored");
        }
        else{
            outputMoving.set(true);
            teamUtil.log("Launching Thread to outputLoadNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    outputLoad(timeout);
                }
            });
            thread.start();
        }

    }


    public void outputLowBucket(){
        intake = teamUtil.robot.intake;
        outtake = teamUtil.robot.outtake;

        if(intake.FlipperInUnload.get()||outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_OUTPUT_CLEAR){
            teamUtil.log("Couldn't Put Output Low Bucket");
        }else{
            outputMoving.set(true);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(LIFT_MIDDLE_BUCKET);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            teamUtil.log("outputLowBucket Completed");
            outputLiftAtBottom.set(false);
            outputMoving.set(false);
        }


    }
    public void outputHighBucket(){
        intake = teamUtil.robot.intake;
        outtake = teamUtil.robot.outtake;

        if(intake.FlipperInUnload.get()||outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_OUTPUT_CLEAR){
            if(intake.FlipperInUnload.get()){
                teamUtil.log("Couldn't Put Output High Bucket Cause of Flipper");
            }
            else{
                teamUtil.log("Outtake Was in the Way Couldn't Go to High Bucket");
            }
        }else{
            outputMoving.set(true);
            bucket.setPosition(BUCKET_TRAVEL);
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(LIFT_TOP_BUCKET);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            teamUtil.log("outputHighBucket Completed"); // TODO: The lift is still moving right now...is this OK?
            outputLiftAtBottom.set(false);
            outputMoving.set(false);
        }


    }


    /*
    //Uses Run Using Encoder to bypass PID deceleration part of run up
    public void outputHighBucketV2(){
        intake = teamUtil.robot.intake;
        outtake = teamUtil.robot.outtake;

        if(intake.FlipperInUnload.get()||outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_OUTPUT_CLEAR){
            if(intake.FlipperInUnload.get()){
                teamUtil.log("Couldn't Put Output High Bucket Cause of Flipper");
            }
            else{
                teamUtil.log("Outtake Was in the Way Couldn't Go to High Bucket");
            }
        }else{
            outputMoving.set(true);
            bucket.setPosition(BUCKET_SAFE);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setPower(LIFT_MAX_VELOCITY);
            while(lift.getCurrentPosition()<LIFT_TOP_BUCKET-200){
                ;
            }
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(LIFT_TOP_BUCKET);
            lift.setVelocity(LIFT_MAX_VELOCITY);

            teamUtil.log("outputHighBucket Completed");
            outputLiftAtBottom.set(false);
            outputMoving.set(false);
        }


    }

     */
}