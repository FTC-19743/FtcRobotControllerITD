package org.firstinspires.ftc.teamcode.assemblies;

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
    double EXTENDER_Y_DEADBAND = 0.3;
    double SLIDER_X_DEADBAND = 0.3;


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
        robot.initCV(false);// TODO: false for competition

        if (!teamUtil.justRanAuto&&!teamUtil.justRanCalibrateRobot) { // Auto already took care of this, so save time and don't move anything!
            robot.calibrate();

        }
        teamUtil.justRanAuto=false;
        teamUtil.justRanCalibrateRobot=false;



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
        //TODO: FIX ALL STATE MANAGEMENT
        waitForStart();

        boolean extenderSliderUnlocked = false;
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

            robot.drive.universalDriveJoystickV2(
                    driverGamepad.gamepad.left_stick_x,
                    driverGamepad.gamepad.left_stick_y,
                    driverGamepad.gamepad.right_stick_x,
                    driverGamepad.gamepad.right_trigger > .5,driverGamepad.gamepad.left_trigger > .5,
                    robot.drive.getHeading());

            if(driverGamepad.wasYPressed()){
                robot.drive.setHeldHeading(0);
            }
            if(driverGamepad.wasAPressed()){
                robot.drive.setHeldHeading(315);
            }
            if(driverGamepad.wasXPressed()){
                robot.drive.setHeldHeading(90);
            }
            if(driverGamepad.wasBPressed()){
                robot.drive.setHeldHeading(270);
            }


            //ARMS GAMEPAD
            //Outake
            if(armsGamepad.wasUpPressed()){
               robot.outtake.deployArm();
            }
            if(armsGamepad.wasDownPressed()){
                robot.outtake.outtakeGrab();
                robot.output.dropSampleOutBackNoWait();
            }
            if(armsGamepad.wasLeftPressed()){
                robot.outtake.outtakeRest();
            }



            if (armsGamepad.wasAPressed()&&!robot.intake.autoSeeking.get()) {
                if(robot.intake.extender.getCurrentPosition()<Intake.EXTENDER_SAFE_TO_UNLOAD_THRESHOLD){
                    robot.intake.unloadNoWait(5000);
                }

            }
            if (armsGamepad.wasRightJoystickFlickedDown()&&!robot.intake.autoSeeking.get()) {
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToUnloadNoWait(3000);
                }else{
                    robot.intake.extenderSafeRetractNoWait(5000);
                }
                extenderSliderUnlocked = false;
            }
            if (driverGamepad.wasLeftTriggerPressed()&&!robot.intake.autoSeeking.get()) {
                robot.intake.goToSeekNoWait(3000);
                extenderSliderUnlocked = true;
            } //ABOVE WAS ARMS GAMEPAD RIGHT FLICK UP


            if ((Math.abs(armsGamepad.gamepad.left_stick_y) > EXTENDER_Y_DEADBAND)&&extenderSliderUnlocked&&!robot.intake.autoSeeking.get()) {
                robot.intake.manualY(armsGamepad.gamepad.left_stick_y);
            }
            if(extenderSliderUnlocked&&!robot.intake.autoSeeking.get()) robot.intake.manualX(armsGamepad.gamepad.left_stick_x);


            if ((armsGamepad.wasBPressed()&&teamUtil.alliance == teamUtil.Alliance.RED)&&extenderSliderUnlocked&&!robot.intake.autoSeeking.get()) { //Grab Red
                robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
                robot.intake.goToSampleAndGrabNoWait(Intake.GO_TO_SAMPLE_AND_GRAB_NO_WAIT_TIMEOUT);  //TODO Tune timeout

            }
            if ((armsGamepad.wasYPressed())&&extenderSliderUnlocked&&!robot.intake.autoSeeking.get()) { //Grab Yellow
                robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
                robot.intake.goToSampleAndGrabNoWait(Intake.GO_TO_SAMPLE_AND_GRAB_NO_WAIT_TIMEOUT); //TODO Tune timeout
            }
            if ((armsGamepad.wasXPressed()&&teamUtil.alliance == teamUtil.Alliance.BLUE)&&extenderSliderUnlocked&&!robot.intake.autoSeeking.get()) { //Grab Blue
                robot.intake.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
                robot.intake.goToSampleAndGrabNoWait(Intake.GO_TO_SAMPLE_AND_GRAB_NO_WAIT_TIMEOUT); //TODO Tune timeout
            }


            //OUTPUT
            if (armsGamepad.wasRightTriggerPressed()) {
                robot.output.dropSampleOutBackNoWait();
            }

            if (armsGamepad.wasLeftTriggerPressed()) {
                robot.output.outputHighBucket();
            }
            if (armsGamepad.wasLeftBumperPressed()) {
                robot.output.outputLoadNoWait(4000);
            }

            //HANG
            if (driverGamepad.wasUpPressed()) {
                robot.hang.extendHangNoWait(4000);
            }
            if (driverGamepad.wasDownPressed()) {
                robot.hang.engageHangNoWait(4000);
            }



            robot.outputTelemetry();
            robot.drive.odo.update();
            telemetry.update();


        }
    }
}
