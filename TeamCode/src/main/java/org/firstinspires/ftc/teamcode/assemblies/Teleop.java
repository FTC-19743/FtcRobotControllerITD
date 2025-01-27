package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetectorV2;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;
    Blinkin blinkin;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean endgame = false;
    double EXTENDER_Y_DEADBAND = 0.3;
    double SLIDER_X_DEADBAND = 0.3;
    boolean lowBucketToggle= false;
    int optionsPresses = 0;
    boolean hangManualControl= false;
    public static boolean initCV= false;



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
        robot.initCV(initCV);// TODO: false for competition

        if (!teamUtil.justRanAuto&&!teamUtil.justRanCalibrateRobot) { // Auto already took care of this, so save time and don't move anything!
            robot.calibrate();

        }
        teamUtil.justRanAuto=false;
        teamUtil.justRanCalibrateRobot=false;



        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();


        robot.drive.setHeading(0);

        optionsPresses=0;
        hangManualControl = false;

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


        robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
        boolean liftDropped = false;
        while (opModeIsActive()){
            driverGamepad.loop();
            armsGamepad.loop();



            ////////// Drive
            if (driverGamepad.gamepad.right_stick_button && driverGamepad.gamepad.left_stick_button) {
                robot.drive.setHeading(0);
            }



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
                robot.dropSampleOutBackAndArmGrabNoWait(3000);
            }
            if(armsGamepad.wasLeftPressed()){
                robot.outtake.outtakeRest();
            }



            if (armsGamepad.wasAPressed()&&!robot.intake.autoSeeking.get()) {
                if(robot.intake.extender.getCurrentPosition()<Intake.EXTENDER_GO_TO_SEEK_THRESHOLD){
                    robot.intake.unloadV2NoWait(true);
                }
                else{
                    robot.intake.extenderSafeRetractNoWait(4000);
                }
            }





            if ((Math.abs(armsGamepad.gamepad.left_stick_y) > EXTENDER_Y_DEADBAND)&&!robot.intake.autoSeeking.get()) {
                robot.intake.manualY(armsGamepad.gamepad.left_stick_y);
            }
            if(!robot.intake.autoSeeking.get()) robot.intake.manualX(armsGamepad.gamepad.left_stick_x);


            if ((armsGamepad.wasBPressed()&&teamUtil.alliance == teamUtil.Alliance.RED)&&!robot.intake.autoSeeking.get()) { //Grab Red
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.RED);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(true);
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }

                  //TODO Tune timeout

            }
            if ((armsGamepad.wasYPressed())&&!robot.intake.autoSeeking.get()) { //Grab Yellow
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.YELLOW);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.goToSampleAndGrabAndLiftToBucketNoWait(true);
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }
            }
            if ((armsGamepad.wasXPressed()&&teamUtil.alliance == teamUtil.Alliance.BLUE)&&!robot.intake.autoSeeking.get()) { //Grab Blue
                robot.intake.setTargetColor(OpenCVSampleDetectorV2.TargetColor.BLUE);
                if((robot.drive.getHeadingODO()>45&&robot.drive.getHeadingODO()<135)||(robot.drive.getHeadingODO()>225&&robot.drive.getHeadingODO()<315)){
                    robot.intake.goToSampleAndGrabNoWaitV3(true);
                }else{
                    robot.intake.goToSampleAndGrabNoWaitV3(false);
                }
            }


            //OUTPUT
            if (armsGamepad.wasRightTriggerPressed()) {
                robot.dropSampleOutBackWithFlipperResetNoWait();
            }
            if(armsGamepad.wasHomePressed()){
                if(lowBucketToggle){
                    lowBucketToggle=false;
                }else{
                    lowBucketToggle=true;

                }
            }
            if (armsGamepad.wasLeftTriggerPressed()) {
                if(!lowBucketToggle){
                    robot.output.outputHighBucketNoWait(3000);
                }else{
                    robot.output.outputLowBucketNoWait(3000);

                }
            }
            if (armsGamepad.wasLeftBumperPressed()) {
                robot.output.outputLoadNoWait(4000);
            }

            //HANG

            if (driverGamepad.wasUpPressed()) {
                robot.hang.extendHang();
            }
            if(armsGamepad.wasOptionsPressed()){
                optionsPresses+=1;
                if(optionsPresses==1){
                    robot.hangPhase1NoWait();
                }if(optionsPresses==2){
                    robot.hangPhase2NoWait();
                    hangManualControl=true;
                }
            }

            if(hangManualControl){

                robot.drive.stopMotors();
                robot.hang.joystickDriveV2(gamepad1.left_stick_x, gamepad1.left_stick_y);
                robot.dropLiftWhenNeeded();
                //break out
                if(driverGamepad.wasHomePressed()){
                    hangManualControl=false;
                    optionsPresses=0;
                }
            }else{
                //drive
                robot.drive.universalDriveJoystickV2(
                        driverGamepad.gamepad.left_stick_x,
                        driverGamepad.gamepad.left_stick_y,
                        driverGamepad.gamepad.right_stick_x,
                        driverGamepad.gamepad.right_trigger > .5,driverGamepad.gamepad.left_trigger > .5,
                        robot.drive.getHeading());
            }






            robot.outputTelemetry();
            robot.drive.odo.update();
            telemetry.addLine("Low Bucket Toggled: " + lowBucketToggle);
            telemetry.addLine("Hang Manual: " + hangManualControl);
            telemetry.update();


        }
    }
}
