package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
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

    static public int A01_PLACE_SPECIMEN_X = 732;
    public static int A02_PLACE_SPECIMEN_Y = -50;
    static public int A03_MOVE_TO_SAMPLE_Y = 873;
    static public int A04_MOVE_TO_SAMPLE_X = 603;
    static public int A05_MOVE_TO_BUCKET_Y = 1084;
    static public int A06_MOVE_TO_BUCKET_X = 119;
    static public int A07_ROBOTHEADING_TO_TURN_TO_BUCKET = 315;
    static public int A08_MOVE_TO_SAMPLE_Y = 1145;
    static public int A09_MOVE_TO_SAMPLE_X = 603;
    static public int A10_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_X = A06_MOVE_TO_BUCKET_X+50;
    static public int A11_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_Y = A05_MOVE_TO_BUCKET_Y-50;
    static public float A12_SPECIMEN_MOTOR_POWER = .3f;





    static public boolean waitingForButtonPress = true;
    public static double OUTAKE_ARM_ENGAGE_VAL = 0;

    static public boolean AA_DEBUG_AUTO = false;
    //public Intake intake;
    //public Output output;
    //public Lift lift;
    //public PixelRelease releaser;
    //public Launcher launcher;

    public double a,b,c,d;

    public boolean keepGoing() {
        if (!AA_DEBUG_AUTO) return true;
        while (true) {
            if (teamUtil.theOpMode.gamepad1.right_bumper) {
                while (teamUtil.theOpMode.gamepad1.right_bumper)
                    ;
                return true;
            }
            if (teamUtil.theOpMode.gamepad1.left_bumper) {
                while (teamUtil.theOpMode.gamepad1.left_bumper)
                    ;
                return false;
            }
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

    public boolean placeSpecimen(long timeout) {
        teamUtil.log("Place Specimen");
        long timeoutTime = System.currentTimeMillis()+timeout;
        outtake.outakearm.setPosition(OUTAKE_ARM_ENGAGE_VAL);
        while(outtake.outakePotentiometer.getVoltage()<Outtake.POTENTIOMETER_ATTACH && teamUtil.keepGoing(timeoutTime)){
        }
        if (System.currentTimeMillis() >= timeoutTime) {
            teamUtil.log("Place Specimen -- TIMED OUT");
            return false;
        }
        teamUtil.log("Place Specimen -- Finished");
        return true;
    }


    public boolean autoV1(boolean button, long buttonTimeout){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));

        drive.setRobotPosition(0,0,0);
        //Get intake and output into positions

        outtake.deployArm();

        // Get close to submersible then mechanically align
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A01_PLACE_SPECIMEN_X,A02_PLACE_SPECIMEN_Y,0,BasicDrive.MIN_END_VELOCITY+1,null,0,4000);
        drive.setMotorPower(A12_SPECIMEN_MOTOR_POWER);
        teamUtil.pause(500);

        if (!placeSpecimen(2000)) {
            teamUtil.log("FAILED to place specimen.  Giving up");
            return false;
        }
        drive.stopMotors();
        if (!keepGoing()) return false;

        // Move over to first yellow sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A04_MOVE_TO_SAMPLE_X,0,0,500,null,0,4000);
        outtake.outtakeRest();
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY,A03_MOVE_TO_SAMPLE_Y,A04_MOVE_TO_SAMPLE_X, 0,0, null,0, 5000);
        intake.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }


        dropSampleInHighBucket();
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY, A08_MOVE_TO_SAMPLE_Y, A04_MOVE_TO_SAMPLE_X,0,0, null,0, 5000);

        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }

        dropSampleInHighBucket();
        drive.stopMotors();

        return true;
    }
    public void resetRobot(){
        outtake.outtakeRest();
        teamUtil.pause(2000);
        intake.goToSafe();
        teamUtil.pause(2000);
        output.outputLoad(4000);
        outtake.secondCalibrate();
        intake.extendersToPosition(Intake.EXTENDER_UNLOAD,4000);
    }

    public void sampleInBucketAndDeploy(){
        intake.goToUnload(3000);
        output.outputHighBucket();
    }

    public void sampleInBucketAndDeployNoWait(){
        teamUtil.log("Launching Thread to outputLoadNoWait");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                sampleInBucketAndDeploy();
            }
        });
        thread.start();
    }

    public void dropSampleInHighBucket(){
        //THIS ASSUMES THAT ROBOT WILL MOVE AFTER THIS METHOD IS CALLED
        sampleInBucketAndDeployNoWait();

        drive.moveTo(BasicDrive.MAX_VELOCITY,A05_MOVE_TO_BUCKET_Y,A06_MOVE_TO_BUCKET_X,A07_ROBOTHEADING_TO_TURN_TO_BUCKET,0, null,0, 5000);
        while(output.lift.getCurrentPosition()<(Output.LIFT_TOP_BUCKET-10)){
            teamUtil.pause(50);
        }
        output.dropSampleOutBack();
        output.outputLoadNoWait(4000);
        drive.moveTo(BasicDrive.MAX_VELOCITY, A11_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_Y, A10_MOVE_TO_SAFE_OUTPUT_LOAD_POSITION_X, A07_ROBOTHEADING_TO_TURN_TO_BUCKET, 300,null,0, 2000);
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

