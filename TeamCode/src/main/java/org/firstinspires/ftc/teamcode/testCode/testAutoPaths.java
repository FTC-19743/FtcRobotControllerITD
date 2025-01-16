package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.BasicDrive;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "testAutoPaths", group = "LinearOpMode")
public class testAutoPaths extends LinearOpMode {

    public static void log(String logString) {
        RobotLog.d("19743LOG:" + Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }
    Robot robot;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean useArms = false;
    public static boolean liveStream = true;
    public static int BLOCKS = 2;
    public static int CYCLES = 0;
    public static boolean useCV = false;

    public static boolean ASCENT = true;


    public long startTime;
    public long elapsedTime;



    private boolean enableLiveView = false;



    public void runOpMode() {
        teamUtil.init(this);
        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        driverGamepad.reset();
        armsGamepad.reset();
        telemetry.addLine("Initializing.  Please wait.");
        telemetry.update();
        robot = new Robot();
        teamUtil.robot = robot;

        robot.initialize();
        if (useCV) {
            telemetry.addLine("Initializing CV.  Please wait.");
            telemetry.update();
            robot.initCV(liveStream);
        }
        telemetry.addLine("Calibrating.  Please wait.");
        telemetry.update();
        robot.calibrate();

        telemetry.addLine("Ready to start");
        telemetry.update();
        robot.drive.setHeading(180);

        driverGamepad.reset();
        armsGamepad.reset();
        waitForStart();

        driverGamepad.reset();
        armsGamepad.reset();
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();
            telemetry.addLine("Alliance: "+ teamUtil.alliance);
            telemetry.addLine("Use Arms: "+ useArms);

            telemetry.addLine("Strafe: "+ robot.drive.odo.getPosY());
            telemetry.addLine("Forward: "+ robot.drive.odo.getPosX());




            if (driverGamepad.wasLeftBumperPressed()) {
                if (teamUtil.alliance== RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = RED;
                }
            }
            if (driverGamepad.wasRightBumperPressed()) {
                useArms = !useArms;
            }


            if(driverGamepad.wasLeftPressed()) {
                robot.resetRobot();
            }
            if(driverGamepad.wasUpPressed()) {
                long startTime = System.currentTimeMillis();
               robot.autoV1Bucket(BLOCKS,ASCENT);
               elapsedTime = System.currentTimeMillis()-startTime;
            }
            if(driverGamepad.wasDownPressed()) {
                robot.specimenCollectBlocks();
            }
            if(driverGamepad.wasRightTriggerPressed()) {
                robot.drive.setRobotPosition(0,0,0);
            }
            if(driverGamepad.wasXPressed()) {
                long startTime = System.currentTimeMillis();
                for(int i = 1; i<=CYCLES;i++){
                    teamUtil.log("Auto V3 Specimen Cycle Number: " + i);
                    robot.specimenCycleV2(i);
                }

                robot.drive.stopMotors();
                elapsedTime = System.currentTimeMillis()-startTime;
            }
            if(driverGamepad.wasYPressed()) {
                long startTime = System.currentTimeMillis();
                robot.autoV1Specimen(BLOCKS,ASCENT);
                elapsedTime = System.currentTimeMillis()-startTime;

            }
            if(driverGamepad.wasBPressed()) {
                long startTime = System.currentTimeMillis();
                robot.autoV2Specimen(BLOCKS);
                elapsedTime = System.currentTimeMillis()-startTime;
            }
            if(driverGamepad.wasAPressed()){
                long startTime = System.currentTimeMillis();
                robot.autoV3Specimen(CYCLES);
                elapsedTime = System.currentTimeMillis()-startTime;
            }
            /*
            if(driverGamepad.wasOptionsPressed()){
                robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
                robot.drive.setRobotPosition(robot.B07_GO_TO_SAMPLE_X,robot.B08_GO_TO_SAMPLE_Y,0);

                if(!robot.intake.goToSampleAndGrab(5000)){
                    teamUtil.log("FAILED to intake sample.  Giving up");
                    break;
                }
                robot.intake.goToUnloadNoWait(5000);
                robot.drive.straightHoldingStrafeEncoder(BasicDrive.MAX_VELOCITY, robot.B11_WALL_SPECIMEN_X, robot.B12_WALL_SPECIMEN_Y,0,200,null,0,4000);
                robot.output.dropSampleOutBackNoWait();
            }

             */

            if(driverGamepad.wasStartPressed()){
                //robot.intake.putFlickerDown();
            }





            robot.drive.odo.update();

            robot.drive.driveMotorTelemetry();
            telemetry.addLine("Running Tests " );
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);
            telemetry.update();
        }
    }
}
