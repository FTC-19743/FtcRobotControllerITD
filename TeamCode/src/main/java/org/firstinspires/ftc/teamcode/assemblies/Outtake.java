package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

public class Outtake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo outakewrist;
    public Servo outakearm;




    public Outtake() {
        teamUtil.log("Constructing Outtake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Outtake");
        //outakewrist = hardwareMap.get(Servo.class,"outakewrist");
        //outakearm = hardwareMap.get(Servo.class,"outakearm");







        teamUtil.log("Intake Outtake");
    }

    public void calibrate(){
        //TODO: IMPLEMENT
    }
}