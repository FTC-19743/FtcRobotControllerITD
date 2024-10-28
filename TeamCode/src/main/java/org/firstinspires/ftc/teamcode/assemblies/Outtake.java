package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
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

    static public float ARM_UP = 0.03f;
    static public float ARM_DOWN = .62f;
    static public float WRIST_GRAB = 0.17f;
    static public float WRIST_RELEASE = .84f;
    static public float ARM_REST = .1f;
    static public float WRIST_REST = 0.5f;



    public Outtake() {
        teamUtil.log("Constructing Outtake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Outtake");
        outakewrist = hardwareMap.get(Servo.class,"outakewrist");
        outakearm = hardwareMap.get(Servo.class,"outakearm");

        teamUtil.log("Intake Outtake");
    }

    public void testWiring() {
        outakearm.setPosition(ARM_UP);
        outakewrist.setPosition(WRIST_GRAB);
    }

    public void calibrate(){
        //TODO: IMPLEMENT
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
}