package org.firstinspires.ftc.teamcode.testCode;

import static org.firstinspires.ftc.teamcode.libs.teamUtil.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


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


        telemetry.addLine("Initializing CV.  Please wait.");
        telemetry.update();
        robot.initialize();
        robot.initCV(liveStream);
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
               robot.autoV1(driverGamepad.wasUpPressed(),1000);
               elapsedTime = System.currentTimeMillis()-startTime;
            }
            if(driverGamepad.wasDownPressed()) {

            }
            if(driverGamepad.wasRightPressed()) {

            }
            if(driverGamepad.wasXPressed()) {


            }
            if(driverGamepad.wasYPressed()) {


            }
            if(driverGamepad.wasBPressed()) {


            }
            if(driverGamepad.wasOptionsPressed()){

            }

            if(driverGamepad.wasStartPressed()){
                //robot.intake.putFlickerDown();
            }







            telemetry.addLine("Running Tests " );
            telemetry.addLine("Last Auto Elapsed Time: " + elapsedTime);

            telemetry.update();
        }
    }
}
