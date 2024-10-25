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

    public boolean details;

    static public int LIFT_MAX = 3070; //TODO find this number and use it in methods
    static public int LIFT_MAX_VELOCITY = 2000;
    static public int LIFT_MIN_VELOCITY = 200;
    static public int LIFT_DOWN = 0;
    static public int LIFT_TOP_BUCKET = 3070; // TODO Determine this number
    static public int LIFT_MIDDLE_BUCKET = 1700; // TODO Determine this number

    static public float BUCKET_DEPLOY = 0.38f;
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
    }

    public void outputTelemetry(){
        telemetry.addLine("Output Lift Position: " + lift.getCurrentPosition());
    }

    public void dropSampleOutBack(){
        bucket.setPosition(BUCKET_DEPLOY);
        teamUtil.log("DropSampleOutBack Completed");
    }
    public void outputLoad(){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bucket.setPosition(BUCKET_RELOAD);
        lift.setTargetPosition(LIFT_DOWN);
        lift.setVelocity(1000);
        teamUtil.log("outputLoad Completed");
        //TODO get rid of Aylas bad lift code
    }
    public void outputLowBucket(){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(LIFT_MIDDLE_BUCKET);
        lift.setVelocity(1000);
        //TODO get rid of Aylas bad lift code
    }
    public void outputHighBucket(){
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setTargetPosition(LIFT_TOP_BUCKET);
        lift.setVelocity(1000);
        //TODO get rid of Aylas bad lift code
    }
}