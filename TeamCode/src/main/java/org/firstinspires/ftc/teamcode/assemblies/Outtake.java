package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config // Makes Static data members available in Dashboard
public class Outtake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo outakewrist;
    public Servo outakearm;
    public AnalogInput outakePotentiometer;


    static public float ARM_UP = 0.22f;
    static public float ARM_DOWN = 0.795f;
    static public float ARM_BUCKET_SAFE = .55f;
    static public float WRIST_GRAB = 0.15f;
    static public float WRIST_RELEASE = .82f;
    static public float ARM_REST = .26f;
    static public float ARM_ENGAGE = 0.2f;
    static public float WRIST_REST = 0.48f;
    static public double POTENTIOMETER_OUTPUT_CLEAR = 2.183;
    static public double POTENTIOMETER_SAFE = 2.38;
    static public double POTENTIOMETER_RELEASE = 2.566;
    static public double POTENTIOMETER_BUCKET_SAFE = 1.5;
    static public double POTENTIOMETER_ATTACH = 3.15;
    static public double POTENTIOMETER_GRAB = .83;
    static public float ARM_START = 0.85f;
    static public float ARM_LEVEL_ONE_ASCENT = 0.13f;




    public Outtake() {
        teamUtil.log("Constructing Outtake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Outtake");
        outakewrist = hardwareMap.get(Servo.class,"outakewrist");
        outakearm = hardwareMap.get(Servo.class,"outakearm");
        outakePotentiometer = hardwareMap.analogInput.get("outakePotentiometer");



        teamUtil.log("Intake Outtake");
    }

    public void testWiring() {
        outakearm.setPosition(ARM_UP);
        outakewrist.setPosition(WRIST_GRAB);
    }

    public void firstCalibrate(){
        outakearm.setPosition(ARM_REST);
        outakewrist.setPosition(WRIST_REST);
        /*
        while(Math.abs(outakePotentiometer.getVoltage()-ARM_REST)>0.1){
        }

         */


    }
    public void secondCalibrate(){
        outakearm.setPosition(ARM_START);
        outakewrist.setPosition(WRIST_REST);


    }
    public void deployArm(){
        outakearm.setPosition(ARM_UP);
        outakewrist.setPosition(WRIST_RELEASE);
    }

    public void outtakeGrab(){
        outakearm.setPosition(ARM_DOWN);
        outakewrist.setPosition(WRIST_GRAB);
    }

    public void outtakeRest(){
        outakearm.setPosition(ARM_REST);
        outakewrist.setPosition(WRIST_REST);
    }

    public void setArmLevelOneAscent(){
        outakearm.setPosition(ARM_LEVEL_ONE_ASCENT);
        outakewrist.setPosition(WRIST_REST);
    }

    public void outakeTelemetry(){
        telemetry.addLine("Outtake arm voltage: " + outakePotentiometer.getVoltage());
    }
}