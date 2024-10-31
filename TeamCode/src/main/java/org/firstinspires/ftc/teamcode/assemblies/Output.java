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
    public AtomicBoolean outputMoving = new AtomicBoolean(false);
    public AtomicBoolean outputLiftAtBottom = new AtomicBoolean(true);

    public boolean details;

    static public int LIFT_MAX = 3070; //TODO find this number and use it in methods
    static public int LIFT_MAX_VELOCITY = 2000;
    static public int LIFT_MIN_VELOCITY = 200;
    static public int LIFT_DOWN = 2;
    static public int LIFT_TOP_BUCKET = 3070; // TODO Determine this number
    static public int LIFT_MIDDLE_BUCKET = 1700; // TODO Determine this number

    static public float BUCKET_DEPLOY_AT_BOTTOM = 0.42f;
    static public float BUCKET_DEPLOY_AT_TOP = 0.44f;

    static public float BUCKET_RELOAD = 0.7f;



    public Output() {
        teamUtil.log("Constructing Output");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Output");
        lift = hardwareMap.get(DcMotorEx.class,"lift");
        bucket = hardwareMap.get(Servo.class,"bucket");

        lift.setDirection(DcMotor.Direction.REVERSE);
        teamUtil.log("Intake Output");
    }
    public void calibrate(){
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
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        teamUtil.log("Calibrate Intake Final: Extender: "+lift.getCurrentPosition());
        outputLiftAtBottom.set(true);
    }

    public void outputTelemetry(){
        telemetry.addLine("Output Lift Position: " + lift.getCurrentPosition());
    }

    public void dropSampleOutBack(){
        if(outputLiftAtBottom.get()){
            bucket.setPosition(BUCKET_DEPLOY_AT_BOTTOM);

        }else{
            bucket.setPosition(BUCKET_DEPLOY_AT_TOP);

        }
        teamUtil.log("DropSampleOutBack Completed");
    }
    public void outputLoad(long timeout){
        intake = teamUtil.robot.intake;
        if(intake.FlipperInUnload.get()){
            teamUtil.log("Couldn't run Output Load Because Flipper In Unload");
        }
        else{
            outputMoving.set(true);
            bucket.setPosition(BUCKET_RELOAD);
            teamUtil.log("Go To Load: Running to Bottom");
            lift.setTargetPosition(LIFT_DOWN);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            long timeOutTime2 = System.currentTimeMillis() + timeout;
            while (teamUtil.keepGoing(timeOutTime2)&&lift.getCurrentPosition() > LIFT_DOWN+10) {
            }
            lift.setVelocity(0);
            outputMoving.set(false);
            outputLiftAtBottom.set(true);
        }


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

        if(intake.FlipperInUnload.get()){
            teamUtil.log("Couldn't Put Output Low Bucket");
        }else{
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(LIFT_MIDDLE_BUCKET);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            teamUtil.log("outputLowBucket Completed");
            outputLiftAtBottom.set(false);
        }


    }
    public void outputHighBucket(){
        intake = teamUtil.robot.intake;

        if(intake.FlipperInUnload.get()){
            teamUtil.log("Couldn't Put Output High Bucket");
        }else{
            lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            lift.setTargetPosition(LIFT_TOP_BUCKET);
            lift.setVelocity(LIFT_MAX_VELOCITY);
            teamUtil.log("outputHighBucket Completed");
            outputLiftAtBottom.set(false);
        }


    }
}