package org.firstinspires.ftc.teamcode.assemblies;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.DelegatingCaptureSequence;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

@TeleOp(name = "Teleop", group = "LinearOpMode")
public class Teleop extends LinearOpMode {

    Robot robot;

    Blinkin blinkin;
    TeamGamepad driverGamepad;
    TeamGamepad armsGamepad;
    boolean endgame = false;

    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;

    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);

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

        VisionPortal arduPortal;

        OpenCVSampleDetector sampleDetector = new OpenCVSampleDetector();
        //sampleDetector.init(); // TODO Last year's code never called init on these processors...
        sampleDetector.viewingPipeline = true;

        CameraName arducam = (CameraName)hardwareMap.get(WebcamName.class, "arducam");
        CameraCharacteristics chars = arducam.getCameraCharacteristics();
        teamUtil.log(arducam.toString());
        teamUtil.log("WebCam: "+(arducam.isWebcam() ? "true" : "false"));
        teamUtil.log("Unknown: "+(arducam.isUnknown() ? "true" : "false"));
        teamUtil.log(chars.toString());

        teamUtil.log("Setting up ArduCam VisionPortal");
        VisionPortal.Builder armBuilder = new VisionPortal.Builder();
        armBuilder.setCamera(arducam);
        armBuilder.enableLiveView(true);
        //}
        // Can also set resolution and stream format if we want to optimize resource usage.
        armBuilder.setCameraResolution(arduSize);
        //armBuilder.setStreamFormat(TBD);

        armBuilder.addProcessor(sampleDetector);

        arduPortal = armBuilder.build();

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

        sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);

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

            //if(driverGamepad.wasAPressed()){
            //robot.drive.setHeldHeading(robot.driverSide());
            //}
            if(driverGamepad.wasYPressed()||driverGamepad.wasAPressed()){
                robot.drive.setHeldHeading(robot.fieldSide());
            }
            if(driverGamepad.wasXPressed()||driverGamepad.wasBPressed()){
                robot.drive.setHeldHeading(180);
            }

            robot.outputTelemetry();
            telemetry.addLine("Found One : "+sampleDetector.foundOne);
            telemetry.addLine("Rect Angle : "+sampleDetector.rectAngle);
            telemetry.addLine("Rect Center X : "+sampleDetector.rectCenterXOffset);
            telemetry.addLine("Rect Center Y : "+sampleDetector.rectCenterYOffset);





            telemetry.update();


        }
    }
}
