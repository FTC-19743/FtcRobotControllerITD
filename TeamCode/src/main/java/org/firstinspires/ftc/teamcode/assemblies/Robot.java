package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
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
    public Blinkin blinkin;

    public static int PICK_UP_HOOKS_PAUSE_1 = 450;
    public static int PICK_UP_HOOKS_PAUSE_2 = 300;
    public static int PICK_UP_HOOKS_PAUSE_3 = 250;
    public static int PICK_UP_HOOKS_PAUSE_4 = 1000;

    public static int READY_TO_PLACE_HOOKS_PAUSE_1 = 1000;
    public static int READY_TO_PLACE_HOOKS_VELOCITY = 1400;
    public static int PLACE_HOOKS_VELOCITY = 400;


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
    static public int A13_SPECIMEN_END_VELOCITY = 500;
    static public long A14_SPECIMEN_PAUSE = 250;
    static public int A15_TRUSS_MOVEMENT_X = 1333;
    static public int A16_TRUSS_MOVEMENT_Y = 500;
    static public int A17_TRUSS_MOVEMENT_X = 1333;
    static public int A18_TRUSS_MOVEMENT_Y = 236;
    static public int A19_GO_TO_TRUSS_END_VELOCITY = 500;
    static public int A20_BACKOUT_FROM_TRUSS_END_VELOCITY = 500;
    static public float A21_LEVEL_ONE_ASCENT_DRIVE_POWER = 0.3f;
    public static int A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME = 100;


    static public int B01_PLACE_SPECIMEN_X = 760;
    public static int B02_PLACE_SPECIMEN_Y = 120;
    public static int B02a_TRANSITION_VELOCITY = 500;
    static public int B03_END_VELOCITY_SPECIMEN = 200;
    static public float B04_SPECIMEN_MOTOR_POWER = 0.3f;
    static public int B05_SPECIMEN_PAUSE = 250;
    static public int B06_END_VELOCITY_SPECIMEN_BACKUP = 500;
    public static int B07_GO_TO_SAMPLE_X = 600;
    public static int B08_GO_TO_SAMPLE_Y = -1003;


    public static int B09_END_VELOCITY_SEEK_AFTER_BACKUP = 0;
    public static int B10_END_VELOCITY_SPECIMEN = 500;
    public static int B11_WALL_SPECIMEN_X = 100;
    public static int B12_WALL_SPECIMEN_Y = -900;
    public static float B13_SPECIMENDROP_MOTOR_POWER = 0.1f;


    public static int C0a_FAST_STRAFE_ADJUST = 250;
    public static int C0a_FAST_STRAIGHT_ADJUST1 = 100;
    public static int C0a_FAST_STRAIGHT_ADJUST2 = 200;
    public static int C0a_FAST_REVERSE_ADJUST = 50;
    public static int C0a_SLOW_STRAFE_ADJUST = 50;
    static public int C01_PLACE_SPECIMEN_X = 760;
    public static int C02_PLACE_SPECIMEN_Y = 120;
    public static int C03_TRANSITION_VELOCITY_CHILL = 500;
    static public int C04_END_VELOCITY_SPECIMEN = 600;
    static public int C05_SPECIMEN_PAUSE = 250;
    static public int C06_CLEAR_SUB_X = 600;
    static public int C07_CLEAR_SUB_Y = -660;
    public static int C08_TRANSITION_VELOCITY_FAST = 1500;
    static public int C09_CLEAR_SAMPLE_X = 1100;
    static public int C10_SAMPLE_1_Y = -900;
    static public int C10_SAMPLE_Y_ADJUST = 50;
    static public int C11_DROP_SAMPLE_X = 250;
    public static int C12_TRANSITION_VELOCITY_REVERSE = 1000;
    static public int C13_SAMPLE_2_Y = C10_SAMPLE_1_Y-180;
    static public int C14_SAMPLE_3_Y = C13_SAMPLE_2_Y-170;
    static public int C15_BACK_OUT_OBSERVATION_ZONE = 550;

    public static int D0a_FAST_STRAFE_ADJUST = 250;
    public static int D0a_FAST_STRAIGHT_ADJUST1 = 100;
    public static int D0a_FAST_STRAIGHT_ADJUST2 = 200;
    public static int D0a_FAST_REVERSE_ADJUST = 50;
    static public int D00_WALL_TO_MIDFIELD_X = 199;
    static public int D01_WALL_TO_MIDFIELD_Y = -50;
    public static int D02_TRANSITION_VELOCITY_FAST = 2000;
    static public int D03_MIDFIELD_TO_CHAMBER_X = 772;
    static public int D04_MIDFIELD_TO_CHAMBER_Y = 100;
    static public int D05_CHAMBER_TO_MIDFIELD_X = 657;
    static public int D06_CHAMBER_TO_MIDFIELD_Y = 47;
    static public int D07_SPECIMEN_PAUSE = 250;
    static public int D08_PREPARE_FOR_PICKUP_X=370;
    static public int D09_PREPARE_FOR_PICKUP_Y=-732;
    public static int D10_TRANSITION_VELOCITY_SLOW = 750;
    static public int D11_PICKUP_Y = -836;
    static public int D12_PICKUP_X = 75;
    static public int D13_GRAB_SPECIMEN_END_VELOCITY = 300;
    static public int D14_SPECIMEN_GRAB_TIME = 500;
    static public int D15_CYCLE_SPECIMEN_ADJUSTMENT = 100;
    static public int D16_WRIST_CALLBACK = -600;









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
        hang.calibrate();

        outtake.firstCalibrate();
        output.calibrate();
        intake.calibrate();
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


    public boolean autoV1Bucket(int blocks, boolean ascent){
        long startTime = System.currentTimeMillis();
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));

        drive.setRobotPosition(0,0,0);
        //Get intake and output into positions

        outtake.deployArm();
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLET);

        // Get close to submersible then mechanically align
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A01_PLACE_SPECIMEN_X,A02_PLACE_SPECIMEN_Y,0,A13_SPECIMEN_END_VELOCITY,null,0,4000);
        drive.setMotorPower(A12_SPECIMEN_MOTOR_POWER);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.VIOLETHEARTBEAT);
        teamUtil.pause(A14_SPECIMEN_PAUSE);
        teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);

        /*
        if (!placeSpecimen(2000)) {
            teamUtil.log("FAILED to place specimen.  Giving up");
            return false;
        }

         */

        drive.stopMotors();

        // Move over to first yellow sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A04_MOVE_TO_SAMPLE_X,0,0,500,null,0,4000);
        outtake.outtakeRest();
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY,A03_MOVE_TO_SAMPLE_Y,A04_MOVE_TO_SAMPLE_X, 0,0, null,0, 5000);
        intake.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
        /*
        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }

         */


        dropSampleInHighBucket(1);
        if(blocks==1){
            teamUtil.log("Stopped After 1 Block");

            if(ascent){
                outtake.setArmLevelOneAscent();
                drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
                drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);
                drive.setMotorPower(A21_LEVEL_ONE_ASCENT_DRIVE_POWER);
                teamUtil.pause(A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME);
                drive.stopMotors();

            }
            else{
                drive.stopMotors();
            }

            return true;
        }
        intake.goToSeekNoWait(2000);
        drive.moveTo(BasicDrive.MAX_VELOCITY, A08_MOVE_TO_SAMPLE_Y, A04_MOVE_TO_SAMPLE_X,0,0, null,0, 5000);

        /*
        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }

         */

        dropSampleInHighBucket(2);
        if(blocks==2){
            teamUtil.log("Stopped After 2 Blocks");
            if(ascent){
                outtake.setArmLevelOneAscent();
                drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
                drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);
                drive.setMotorPower(A21_LEVEL_ONE_ASCENT_DRIVE_POWER);
                teamUtil.pause(A22_LEVEL_ONE_ASCENT_DRIVE_POWER_TIME);
                drive.stopMotors();


            }
            else{
                drive.stopMotors();
            }

            return true;
        }
        drive.stopMotors();

        drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A19_GO_TO_TRUSS_END_VELOCITY,null,0,4000);
        intake.goToSeekNoWait(2000);

        drive.moveTo(BasicDrive.MAX_VELOCITY, A18_TRUSS_MOVEMENT_Y,A17_TRUSS_MOVEMENT_X,270,0,null,0,4000);
        /*
        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }

         */
        sampleInBucketAndDeployNoWait();
        drive.moveTo(BasicDrive.MAX_VELOCITY, A16_TRUSS_MOVEMENT_Y,A15_TRUSS_MOVEMENT_X,270,A20_BACKOUT_FROM_TRUSS_END_VELOCITY,null,0,4000);
        dropSampleInHighBucket(3);
        if(blocks==3){
            teamUtil.log("Stopped After 3 Blocks");

            return true;
        }


        return true;
    }

    public void specimenCollectBlocks() {
        drive.setRobotPosition(0,0,0);
        long startTime = System.currentTimeMillis();
        //Drive to the submersible while moving a bit to the left
        outtake.deployArm();
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C01_PLACE_SPECIMEN_X, C02_PLACE_SPECIMEN_Y,0,C04_END_VELOCITY_SPECIMEN,null,0,4000);
        teamUtil.pause(C05_SPECIMEN_PAUSE); // give it time to click in

        // Back up to clear sub
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C06_CLEAR_SUB_X, C02_PLACE_SPECIMEN_Y,0,C03_TRANSITION_VELOCITY_CHILL,null,0,4000);
        outtake.outtakeGrab();
        // strafe over to clear sub on other side
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, C07_CLEAR_SUB_Y+C0a_FAST_STRAFE_ADJUST, C06_CLEAR_SUB_X, 0, C08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // drive past samples
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST1, C07_CLEAR_SUB_Y,0,C08_TRANSITION_VELOCITY_FAST,null,0,4000);

        // strafe to sample 1
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY, C10_SAMPLE_1_Y+C10_SAMPLE_Y_ADJUST, C09_CLEAR_SAMPLE_X, 0, C08_TRANSITION_VELOCITY_FAST,null, 0, 2000);

        // push first sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C10_SAMPLE_1_Y,0,C12_TRANSITION_VELOCITY_REVERSE,null,0,4000);

        // head back out to get second sample
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST2, C10_SAMPLE_1_Y,0,C08_TRANSITION_VELOCITY_FAST,null,0,4000);
        drive.strafeToTarget(270,0,C08_TRANSITION_VELOCITY_FAST, C13_SAMPLE_2_Y+C10_SAMPLE_Y_ADJUST,2000);

        // push second sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C13_SAMPLE_2_Y,0,C12_TRANSITION_VELOCITY_REVERSE,null,0,4000);

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C15_BACK_OUT_OBSERVATION_ZONE, C13_SAMPLE_2_Y,0,C08_TRANSITION_VELOCITY_FAST,null,0,4000);

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, D12_PICKUP_X,C13_SAMPLE_2_Y,0,D13_GRAB_SPECIMEN_END_VELOCITY,null,0,4000);
        teamUtil.pause(D14_SPECIMEN_GRAB_TIME);
        // head back out for 3rd sample
        /*
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C09_CLEAR_SAMPLE_X- C0a_FAST_STRAIGHT_ADJUST2, C13_SAMPLE_2_Y,0,C08_TRANSITION_VELOCITY_FAST,null,0,4000);
        drive.strafeToTarget(270,0,C08_TRANSITION_VELOCITY_FAST, C14_SAMPLE_3_Y+C10_SAMPLE_Y_ADJUST,2000);

        // push 3rd sample to observation zone
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, C11_DROP_SAMPLE_X+C0a_FAST_REVERSE_ADJUST, C14_SAMPLE_3_Y,0,C12_TRANSITION_VELOCITY_REVERSE,null,0,4000);
         */
        drive.stopMotors(); // temp

    }

    public boolean autoV1Specimen(int blocks,boolean ascent){
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));

        drive.setRobotPosition(0,0,0);
        long startTime = System.currentTimeMillis();

        outtake.deployArm();

        //First move to gets robot over to side in order get to submersible fast enough
        //drive.moveTo(BasicDrive.MAX_VELOCITY,B02_PLACE_SPECIMEN_Y,B02_PLACE_SPECIMEN_Y,0,B10_END_VELOCITY_SPECIMEN,null,0,5000);
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, B01_PLACE_SPECIMEN_X, B02_PLACE_SPECIMEN_Y, 0, B03_END_VELOCITY_SPECIMEN, null, 0,3000);
        drive.setMotorPower(B04_SPECIMEN_MOTOR_POWER);
        teamUtil.pause(B05_SPECIMEN_PAUSE);
        drive.stopMotors();

        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, A04_MOVE_TO_SAMPLE_X, B02_PLACE_SPECIMEN_Y,0,500,null,0,4000);
        outtake.outtakeGrab();
        drive.moveTo(BasicDrive.MAX_VELOCITY,B08_GO_TO_SAMPLE_Y,B07_GO_TO_SAMPLE_X,0,B09_END_VELOCITY_SEEK_AFTER_BACKUP,null,0,5000);

        //intake.goToSeekNoWait(5000);

        if(teamUtil.alliance == RED) intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
        else intake.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
        /*
        if(!intake.goToSampleAndGrab(5000)){
            teamUtil.log("FAILED to intake sample.  Giving up");
            return false;
        }

         */
        intake.goToUnload(5000);
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, B11_WALL_SPECIMEN_X, B12_WALL_SPECIMEN_Y,0,200,null,0,4000);
        output.dropSampleOutBackNoWait();

        drive.setMotorPower(-B13_SPECIMENDROP_MOTOR_POWER);
        teamUtil.pause(1000);
        drive.stopMotors();




        return true;
    }

    public boolean autoV2Specimen(int cycles){
        teamUtil.log("Running Auto.  Alliance: " + (teamUtil.alliance == RED ? "RED" : "BLUE"));
        drive.setRobotPosition(0,0,0);
        specimenCollectBlocks();
        for(int i = 1; i<=cycles;i++){
            teamUtil.log("Auto V2 Specimen Cycle Number: " + i);
            specimenCycle(i);
        }
        drive.stopMotors();
        return true;
    }

    public boolean specimenCycle(int cycles){
        outtake.outakearm.setPosition(Outtake.ARM_UP);

        BasicDrive.MIN_STRAFE_START_VELOCITY = 2000;
        BasicDrive.MIN_START_VELOCITY = 1000;
        //Moves robot from the observation zone to the middle of the field
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,D01_WALL_TO_MIDFIELD_Y,D00_WALL_TO_MIDFIELD_X,0,D02_TRANSITION_VELOCITY_FAST,
                new BasicDrive.ActionCallback() {
                    @Override
                    public void action() {
                        outtake.outakewrist.setPosition(Outtake.WRIST_RELEASE);
                    }
                },D16_WRIST_CALLBACK,5000);
        outtake.deployArm();

        //moves robot from the middle of the field to scoring the specimen
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D03_MIDFIELD_TO_CHAMBER_X,D04_MIDFIELD_TO_CHAMBER_Y+(D15_CYCLE_SPECIMEN_ADJUSTMENT*(cycles-1)),0,C04_END_VELOCITY_SPECIMEN,null,0,4000);
        teamUtil.pause(D07_SPECIMEN_PAUSE);

        //moves robot out of the way of the submersible
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D05_CHAMBER_TO_MIDFIELD_X,D06_CHAMBER_TO_MIDFIELD_Y,0,D02_TRANSITION_VELOCITY_FAST,null,0,4000);
        outtake.outtakeGrab();

        //moves robot into postion to drive forward to grab next specimen
        drive.strafeHoldingStraightEncoder(BasicDrive.MAX_VELOCITY,D09_PREPARE_FOR_PICKUP_Y,D08_PREPARE_FOR_PICKUP_X,0, D10_TRANSITION_VELOCITY_SLOW,null,0,4000);

        //moves robot to wall for grab
        drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY,D12_PICKUP_X,D11_PICKUP_Y,0,D13_GRAB_SPECIMEN_END_VELOCITY,null,0,4000);
        teamUtil.pause(D14_SPECIMEN_GRAB_TIME);
        BasicDrive.MIN_STRAFE_START_VELOCITY = 500;
        BasicDrive.MIN_START_VELOCITY = 300;
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
        intake.unload();
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

    public void dropSampleInHighBucket(int cycle){
        //THIS ASSUMES THAT ROBOT WILL MOVE AFTER THIS METHOD IS CALLED
        if(cycle<3){
            sampleInBucketAndDeployNoWait();
        }

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

    public boolean dropSampleOutBackAndArmGrab(long timeout){
        //TODO Implement Timeout

        output.dropSampleOutBackNoWait();
        outtake.outtakeGrab();
        return true;
    }

    public void dropSampleOutBackAndArmGrabNoWait(long timeout){
        if(output.outputMoving.get()){
            teamUtil.log("WARNING: Attempt to dropSampleOutBackAndArmGrabNoWait while output is moving--ignored");
        }
        else{
            teamUtil.log("Launching Thread to dropSampleOutBackNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    dropSampleOutBackAndArmGrab(timeout);
                }

            });
            thread.start();
        }
    }

    public void pickUpHooks(){
        intake.flipper.setPosition(Intake.FLIPPER_SAFE);
        outtake.outakearm.setPosition(Outtake.ARM_ENGAGE);
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY);
        hang.extendHang();


        output.lift.setTargetPosition(Output.LIFT_SAFE_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_1);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_2);
        output.lift.setTargetPosition(Output.LIFT_PICKUP_FOR_HOOK_HOLDER);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_3);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_GRAB);
        teamUtil.pause(PICK_UP_HOOKS_PAUSE_4);

    }

    public void readyToPlaceHooks(){
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_READY);
        teamUtil.pause(READY_TO_PLACE_HOOKS_PAUSE_1);
        output.lift.setVelocity(READY_TO_PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ABOVE_BAR);
        hang.hook_grabber.setPosition(Hang.HOOKGRABBER_DEPLOY);
    }

    public void placeHooks(){
        output.lift.setVelocity(PLACE_HOOKS_VELOCITY);
        output.lift.setTargetPosition(Output.LIFT_ONTO_BAR);
        while (output.lift.getCurrentPosition() > Output.LIFT_ONTO_BAR+10) { // wait for hooks to be released
            teamUtil.pause(50);
        }
        output.lift.setVelocity(Output.LIFT_MAX_VELOCITY); // Run to near bottom
        output.lift.setTargetPosition(Output.LIFT_DOWN+30);
        while (output.lift.getCurrentPosition() > Output.LIFT_DOWN+40) {
            teamUtil.pause(50);
        }
        output.lift.setVelocity(0); // Turn off lift motor at bottom
        output.bucket.setPosition(Output.BUCKET_DEPLOY_AT_BOTTOM); // rotate bucket to avoid string while tensioning

    }

    public void getReadyToHang() {
        pickUpHooks();
        readyToPlaceHooks();
    }

    public void getReadyToHangNoWait() {
        if (hang.hangMoving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to getReadyToHang while hang is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to getReadyToHang");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    getReadyToHang();
                }
            });
            thread.start();
        }

    }

    public void placeHooksNoWait() {
        if (hang.hangMoving.get()) { // Intake is already moving in another thread
            teamUtil.log("WARNING: Attempt to placeHooks while hang is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to placeHooks");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    placeHooks();
                }
            });
            thread.start();
        }

    }


}

