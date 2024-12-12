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
public class Hang {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    public Servo pulley_left;
    public Servo pulley_right;
    public Servo hook_grabber;



    public AtomicBoolean hangMoving = new AtomicBoolean(false);
    public boolean details = true;

    public static int HANG_EXTEND = 3100;
    public static int HANG_ENGAGE = 800;
    public static int HANG_VELOCITY = 3000;

    public static float PULLEYLEFT_STOW = 0.4f;
    public static float PULLEYLEFT_HANG = 0.44f;
    public static float PULLEYLEFT_EXTEND = 0.86f;

    public static float PULLEYRIGHT_STOW = 0.61f;
    public static float PULLEYRIGHT_HANG = 0.54f;
    public static float PULLEYRIGHT_EXTEND = 0.17f;

    public static float HOOKGRABBER_STOW = 0f;
    public static float HOOKGRABBER_GRAB = 0.12f;
    public static float HOOKGRABBER_DEPLOY = 0.9f;
    public static float HOOKGRABBER_RELEASE = 1f;





    public Hang() {
        teamUtil.log("Constructing Hang");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Hang");
        pulley_left = hardwareMap.get(Servo.class,"pulleyleft");
        pulley_right = hardwareMap.get(Servo.class,"pulleyright");
        hook_grabber = hardwareMap.get(Servo.class,"hookgrabber");



    }

    public void calibrate(){
        stowHookGrabber();
    }



    public void outputTelemetry(){
        //telemetry.addLine("Hang Current Position: " +hang.getCurrentPosition());
    }

    public void stowHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_STOW);
    }

    public void grabHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_GRAB);
    }

    public void deployHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_DEPLOY);
    }

    public void releaseHookGrabber(){
        hook_grabber.setPosition(HOOKGRABBER_RELEASE);
    }

    public void extendHang(){
        if (details) teamUtil.log("Extending Hang");
        hangMoving.set(true);
        pulley_left.setPosition(PULLEYLEFT_EXTEND);
        pulley_right.setPosition(PULLEYRIGHT_EXTEND);

        teamUtil.log("Hang Extended");
        hangMoving.set(false);
    }

    public void engageHang(){
        if (details) teamUtil.log("Extending Hang");
        hangMoving.set(true);
        pulley_left.setPosition(PULLEYLEFT_HANG);
        pulley_right.setPosition(PULLEYRIGHT_HANG);

        teamUtil.log("Hang Extended");
        hangMoving.set(false);
    }

    public void stowHang(){
        if (details) teamUtil.log("Extending Hang");
        hangMoving.set(true);
        pulley_left.setPosition(PULLEYLEFT_STOW);
        pulley_right.setPosition(PULLEYRIGHT_STOW);

        teamUtil.log("Hang Extended");
        hangMoving.set(false);
    }
}