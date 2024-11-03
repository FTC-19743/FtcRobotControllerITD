package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public BasicDrive drive;
    public Output output;
    public Outtake outtake;
    public Intake intake;
    public Hang hang;

    static public int firstTarget = 742;
    static public int secondTarget = 512;
    static public boolean waitingForButtonPress = true;
    public static double OUTAKE_ARM_ENGAGE_VAL = 0;

    //public Intake intake;
    //public Output output;
    //public Lift lift;
    //public PixelRelease releaser;
    //public Launcher launcher;

    public double a,b,c,d;

    public boolean waitForButtonPress(boolean buttonPress, long timeout){
        if(waitingForButtonPress){
            long startTime = System.currentTimeMillis();
            while(!buttonPress&&System.currentTimeMillis()<(startTime+timeout)){
                teamUtil.log("Button Press Not Done Within Time");

            }
            if(System.currentTimeMillis()>(startTime+timeout)){
                return false;
            }
            else{
                teamUtil.log("Button Press True and Boolean was True");
                return true;
            }
        }else{
            teamUtil.log("Button Press True and Boolean was False");

            return true;
        }
    }

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        outtake = new Outtake();
        intake = new Intake();
        output = new Output();
        hang = new Hang();
        teamUtil.robot = this;


    }

    public void initialize() {
        outtake.initalize();
        drive.initalize();
        output.initalize();
        intake.initialize();
        hang.initalize();
    }
    public void initCV (boolean enableLiveView) {
        intake.initCV(enableLiveView);
    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
        hang.outputTelemetry();
        output.outputTelemetry();
    }

    public void calibrate() {
        outtake.firstCalibrate();

        output.calibrate();
        intake.calibrate();
        hang.calibrate();
        outtake.secondCalibrate();

    }

    public boolean autoV1(boolean button, long buttonTimeout){
        drive.resetHeading();
        //TODO Take out and replace all control hub imu low level drive code with odo heading info
        drive.setRobotPosition(0,0,0);
        //Get intake and output into positions

        outtake.deployArm();

        //TODO Tune timeout vals
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,firstTarget,0,0,BasicDrive.MIN_END_VELOCITY+1,null,0,4000);
        drive.setMotorPower(0.1);
        teamUtil.pause(500);
        outtake.outakearm.setPosition(OUTAKE_ARM_ENGAGE_VAL);
        while(outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_ATTACH){
        }
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,secondTarget,0,0,500,null,0,4000);
        outtake.outtakeRest();

        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY_STRAFE,851,secondTarget,0,0,null,0,4000);

        /*
        if(!waitForButtonPress(button,buttonTimeout)) return false;

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,secondTarget,0,0,500,null,0,4000);

        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY_STRAFE,851,secondTarget,0,0,null,0,4000);

         */


        return true;
    }



    public int fieldSide() { // helper method that returns heading out towards the field
        return teamUtil.alliance == RED ? 90 : 270;
    }

    public int driverSide() { // helper method that returns heading backs towards drivers
        return teamUtil.alliance == RED ? 270 : 90;
    }

    public int audienceSide() {
        return 180;
    }

    public int scoreSide() {
        return 0;
    }


}

