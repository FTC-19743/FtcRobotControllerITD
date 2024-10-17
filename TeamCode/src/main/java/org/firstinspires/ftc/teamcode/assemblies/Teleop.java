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
        if (!teamUtil.justRanAuto) { // Auto already took care of this, so save time.
            robot.calibrate();
        }

        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();





        robot.drive.setHeading(180);
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
                robot.drive.setHeading(180);
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
            if(driverGamepad.wasDownPressed()){
                currentTargetColor+=1;
                if(currentTargetColor>3){
                    currentTargetColor=1;
                    robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);

                }
                else if(currentTargetColor==2){
                    robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
                }else{
                    robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
                }
            }
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

            if(armsGamepad.wasUpPressed()){
                robot.intake.configureCam(OpenCVSampleDetector.APEXPOSURE, OpenCVSampleDetector.AEPRIORITY, OpenCVSampleDetector.EXPOSURE, OpenCVSampleDetector.GAIN, OpenCVSampleDetector.WHITEBALANCEAUTO, OpenCVSampleDetector.TEMPERATURE, OpenCVSampleDetector.AFOCUS, OpenCVSampleDetector.FOCUSLENGTH);
            }

            if(driverGamepad.wasAPressed()){
                robot.intake.testWiring();
                //robot.outtake.testWiring();
            }
            if(driverGamepad.wasUpPressed()){
                robot.intake.goToSample();
                //robot.outtake.testWiring();
            }
            if(driverGamepad.wasLeftPressed()){
                robot.intake.goToSampleAndGrab();
                //robot.outtake.testWiring();
            }
            if(driverGamepad.wasRightPressed()){
                robot.intake.flipAndRotateToSample();
                //robot.outtake.testWiring();
            }
            if(driverGamepad.wasYPressed()||driverGamepad.wasAPressed()){
                robot.drive.setHeldHeading(robot.fieldSide());
            }
            if(driverGamepad.wasXPressed()||driverGamepad.wasBPressed()){
                robot.drive.setHeldHeading(180);
            }
            if(driverGamepad.wasLeftBumperPressed()){
                robot.intake.grab();
            }
            if(driverGamepad.wasRightBumperPressed()){
                robot.intake.release();
                teamUtil.pause(1000);
                robot.intake.sweeper.setPosition(SWEEPER_READY);
            }
            if(driverGamepad.wasLeftTriggerPressed()){
                robot.intake.flipper.setPosition(FLIPPER_GRAB);
            }
            if(driverGamepad.wasRightTriggerPressed()){
                robot.intake.flipper.setPosition(FLIPPER_READY);
            }
            if (armsGamepad.wasRightBumperPressed()){
                robot.intake.sampleDetector.nextView();
            }
            robot.outputTelemetry();
            telemetry.addLine("Current Color: "+ robot.intake.sampleDetector.targetColor);
            telemetry.addLine("Found One : "+robot.intake.sampleDetector.foundOne);
            telemetry.addLine("Rect Angle : "+robot.intake.sampleDetector.rectAngle);
            telemetry.addLine("Rect Center X : "+robot.intake.sampleDetector.rectCenterXOffset);
            telemetry.addLine("Rect Center Y : "+robot.intake.sampleDetector.rectCenterYOffset);




            telemetry.update();


        }
    }
}
