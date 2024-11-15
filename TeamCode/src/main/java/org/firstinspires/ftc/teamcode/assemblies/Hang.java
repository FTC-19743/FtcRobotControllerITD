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

    public DcMotorEx hang;

    public AtomicBoolean hangMoving = new AtomicBoolean(false);
    public boolean details = true;

    public static int HANG_EXTEND = 3100;
    public static int HANG_ENGAGE = 800;
    public static int HANG_VELOCITY = 3000;


    public Hang() {
        teamUtil.log("Constructing Hang");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Hang");
        hang = hardwareMap.get(DcMotorEx.class,"hang");


    }
    public void calibrate(){
        hang.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hang.setPower(-.3); // go fast first time
        int lastHangPosition = hang.getCurrentPosition();
        teamUtil.pause(250);
        while (hang.getCurrentPosition() != lastHangPosition) {
            lastHangPosition = hang.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Hang: " + hang.getCurrentPosition());
            teamUtil.pause(50);
        }
        hang.setPower(0);
        teamUtil.pause(1000);
        hang.setPower(-.07); //Go slow second time so it doesn't bounce
        lastHangPosition = hang.getCurrentPosition();
        teamUtil.pause(250);
        while (hang.getCurrentPosition() != lastHangPosition) {
            lastHangPosition = hang.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Hang: " + hang.getCurrentPosition());
            teamUtil.pause(50);
        }
        hang.setPower(0);
        hang.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hang.setTargetPosition(hang.getCurrentPosition());
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        teamUtil.log("Calibrate Hang Final: "+hang.getCurrentPosition());
    }

    public void outputTelemetry(){
        telemetry.addLine("Hang Current Position: " +hang.getCurrentPosition());
    }

    public void extendHang(long timeout){
        if (details) teamUtil.log("Extending Hang");
        long timeout2 = System.currentTimeMillis()+timeout;
        hangMoving.set(true);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(HANG_EXTEND);
        hang.setVelocity(HANG_VELOCITY);
        while(Math.abs(hang.getCurrentPosition()-HANG_EXTEND)>20&&System.currentTimeMillis()<timeout2){
            if(details)teamUtil.log("Hang Encoder Position: " + hang.getCurrentPosition());
            teamUtil.pause(20);
        }
        teamUtil.log("Hang Extended");
        hangMoving.set(false);
    }

    public void engageHang(long timeout){
        if (details) teamUtil.log("Engaging Hang");
        long timeout2 = System.currentTimeMillis()+timeout;
        hangMoving.set(true);
        hang.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hang.setTargetPosition(HANG_ENGAGE);
        hang.setVelocity(HANG_VELOCITY);
        while(Math.abs(hang.getCurrentPosition()-HANG_ENGAGE)>20&&System.currentTimeMillis()<timeout2){
            if(details)teamUtil.log("Hang Encoder Position: " + hang.getCurrentPosition());
            teamUtil.pause(20);
        }
        teamUtil.log("Hang Engaged");
        hangMoving.set(false);
    }

    public void extendHangNoWait(long timeout){
        if(hangMoving.get()){
            teamUtil.log("WARNING: Attempt to extendHang while hang is moving--ignored");
        }
        else{
            hangMoving.set(true);
            teamUtil.log("Launching Thread to ExtendHangNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    extendHang(timeout);
                }
            });
            thread.start();
        }
    }

    public void engageHangNoWait(long timeout){
        if(hangMoving.get()){
            teamUtil.log("WARNING: Attempt to engage Hang while Hang is moving--ignored");
        }
        else{
            hangMoving.set(true);
            teamUtil.log("Launching Thread to engageHangNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    engageHang(timeout);
                }
            });
            thread.start();
        }
    }


}