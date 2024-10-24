package org.firstinspires.ftc.teamcode.assemblies;

import static org.firstinspires.ftc.teamcode.assemblies.Intake.FLIPPER_GRAB;
import static org.firstinspires.ftc.teamcode.assemblies.Intake.FLIPPER_READY;
import static org.firstinspires.ftc.teamcode.assemblies.Intake.SWEEPER_READY;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;


@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;

    Blinkin blinkin;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean endgame = false;



    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;






    /*
    public void loopRunTimeCalculate(int loopNumber,boolean button){
        long startTime=0;
        long endTime=0;
        int loopAmount=0;
        int buttonPressNumber=0;
        if(button&&buttonPressNumber==0){
            buttonPressNumber=1;
            startTime=System.currentTimeMillis();
        }
        if(button&&buttonPressNumber==1){
            loopAmount=loopNumber;
            endTime=System.currentTimeMillis();
        }
        long totalRunTime = endTime-startTime;
        long loopTime = totalRunTime/loopAmount;

        //TODO: take away (only for testing)
        telemetry.addLine("Button Press Number" + buttonPressNumber);

        teamUtil.log("Loop Time" + loopTime);


    }

     */
    //tentative test code


    public void runOpMode() {
        teamUtil.init(this);



        driverGamepad = new TeamGamepad();
        driverGamepad.initilize(true);
        armsGamepad = new TeamGamepad();
        armsGamepad.initilize(false);
        robot = new Robot();
        robot.initialize();
        robot.initCV(true); // false for competition
        if (!teamUtil.justRanAuto) { // Auto already took care of this, so save time.
            robot.calibrate();
        }
        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();





        robot.drive.setHeading(0);
        while (!opModeIsActive()) {
            driverGamepad.loop();
            if(driverGamepad.wasRightBumperPressed()||driverGamepad.wasLeftBumperPressed()){
                if(teamUtil.alliance == teamUtil.Alliance.BLUE){
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }else{
                    teamUtil.alliance= teamUtil.Alliance.BLUE;
                }
            }
            telemetry.addLine("Ready to start");
            telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
            telemetry.update();
        }

        waitForStart();

        int currentTargetColor=1;
        robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();

            if(driverGamepad.wasHomePressed()){
                endgame=true;
            }

            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(0);
            }
            /*
            if (teamUtil.alliance == teamUtil.Alliance.RED) { // Make this work for Red and Blue Alliances
                robot.drive.universalDriveJoystick(
                        driverGamepad.gamepad.left_stick_y,
                        -driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,
                        robot.drive.getHeading());
            } else {
                robot.drive.universalDriveJoystick(
                        -driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,
                        robot.drive.getHeading());
            }

             */

            if (teamUtil.alliance == teamUtil.Alliance.RED) { // Make this work for Red and Blue Alliances
                robot.drive.universalDriveJoystickV2(
                        driverGamepad.gamepad.left_stick_y,
                        -driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeading());
            } else {
                robot.drive.universalDriveJoystickV2(
                        -driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5, driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeading());
            }

            robot.outputTelemetry();
            telemetry.update();


        }
    }
}
