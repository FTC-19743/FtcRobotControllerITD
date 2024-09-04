package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.assemblies.BasicDrive;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Test Drive", group = "Test Code")
public class CalibrateDrive extends LinearOpMode {

    BasicDrive drive;
    int testVelocity = 1000;
    int testDistance = 100;
    double HEADING = 0;
    int SECONDS = 3;


    public enum Ops {Test_Wiring,
        Move_No_Acc_Heading,
        Move_No_Acc_With_Heading,
        Find_Max_Forward,
        Find_Max_Left,
        Brake_Test_Forward,
        Brake_Test_Right,
        Test_Spins,
        Test_Move_CMs,
        Test_Holding_Target,
        Test_Move_To,
        Move_CMs_Test,
        Move_Encoder_Target_Test};
    public static Ops AAOP = Ops.Test_Wiring;
    public static int botX = 72;
    public static int botY = 72;
    public static double AMPLITUDE = 10;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.5;

    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();

    private void doAction() {
        switch (AAOP) {
            case Test_Wiring : testDriveMotorWiring();break;
            case Move_No_Acc_Heading : moveNoAccelerateNoHeadingControl();break;
            case Move_No_Acc_With_Heading : moveNoAccelerateWithHeadingControl();break;
            case Find_Max_Forward : drive.findMaxVelocity(testDistance);break;
            case Find_Max_Left : drive.findMaxStrafeVelocity(testDistance);break;
            case Move_CMs_Test : goForADriveCMs();break;
            case Move_Encoder_Target_Test : goForADriveTarget();break;
            case Brake_Test_Forward : brakeTestForward();break;
            case Brake_Test_Right : brakeTestRight();break;

        }
    }
    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        teamUtil.init(this);
        teamUtil.alliance = teamUtil.Alliance.RED;
        teamUtil.SIDE=teamUtil.Side.SCORE;

        drive = new BasicDrive();
        drive.initalize();

        gp1.initilize(true); // Game Pads can be plugged into the computer
        gp2.initilize(false);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gp1.loop();
            gp2.loop();
            drive.driveMotorTelemetry();

            // Left bumper toggles Alliance
            if (gp1.wasLeftBumperPressed()) {
                if (teamUtil.alliance == teamUtil.Alliance.RED) {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                }
            }
            // Right bumper resets Heading
            if (gp1.wasRightBumperPressed()) {
                drive.setHeading(0);
            }

            // X makes the selected action happen
            if (gp1.wasXPressed()) {
                doAction();
            }
            if (AAOP==Ops.Test_Spins){
                testSpins();
            } else if (AAOP==Ops.Test_Move_CMs){
                testMoveCMs();
            } else if (AAOP==Ops.Test_Holding_Target){
                testHoldingTarget();
            } else if (AAOP==Ops.Test_Move_To){
                testMoveTo();
            }

            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/BatBot.jpg", botX, botY, 18, 18);
            //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            dashboard.sendTelemetryPacket(packet);


            // Graphing stuff and putting stuff in telemetry
            //telemetry.addData("Item", data)); // Anything written like this can be graphed against time.  Multiple items can be graphed together

            telemetry.update();
            sleep(20);
        }
    }

    public void testDriveMotorWiring() {
        drive.setMotorVelocities(drive.MIN_START_VELOCITY,0,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,drive.MIN_START_VELOCITY,0,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,drive.MIN_START_VELOCITY,0);
        teamUtil.pause(1000);
        drive.setMotorVelocities(0,0,0,drive.MIN_START_VELOCITY);
        teamUtil.pause(1000);
        drive.stopMotors();
    }

    public void moveNoAccelerateNoHeadingControl () {
        drive.driveMotorsHeadings(HEADING, drive.getHeading(), testVelocity);
        teamUtil.pause(SECONDS*1000);
        drive.stopMotors();
    }

    public void moveNoAccelerateWithHeadingControl () {
        long doneTime = System.currentTimeMillis() + (int)(SECONDS*1000);
        double heldHeading = drive.getHeading();
        while (System.currentTimeMillis() < doneTime) {
            drive.driveMotorsHeadings(HEADING, heldHeading, testVelocity);
        }
        drive.stopMotors();
    }

    public void testSpins() {
        if (gamepad1.dpad_up) {
            drive.spinToHeading(0);
        }
        if (gamepad1.dpad_down) {
            drive.spinToHeading(180);
        }
        if (gamepad1.dpad_left) {
            drive.spinToHeading(270);
        }
        if (gamepad1.dpad_right) {
            drive.spinToHeading(90);
        }
    }

    public void testMoveCMs() {
        if (gp1.gamepad.dpad_up) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 0, 0,0);
        } else if (gp1.gamepad.dpad_down) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 180, 0,0);
        } else if (gp1.gamepad.dpad_left) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 90, 0,0);
        } else if (gp1.gamepad.dpad_right) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 270, 0,0);
        } else if (gp1.gamepad.y) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 45, 0,0);
        } else if (gp1.gamepad.a) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 225, 0,0);
        } else if (gp1.gamepad.x) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 135, 0,0);
        } else if (gp1.gamepad.b) {
            drive.setHeading(0);
            drive.moveCm(testVelocity, testDistance, 315, 0,0);
        }
    }

    public void testHoldingTarget() {
        if (gamepad1.dpad_up) {
            drive.setHeading(0);
            long startForward = drive.forwardEncoder.getCurrentPosition();
            long forwardTarget = (long) (startForward + drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
            long startStrafe = drive.strafeEncoder.getCurrentPosition();
            drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, forwardTarget,startStrafe,0,0,null,0,3000);
        }
        if (gamepad1.dpad_down) {
            drive.setHeading(0);
            long startForward = drive.forwardEncoder.getCurrentPosition();
            long forwardTarget = (long) (startForward - drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
            long startStrafe = drive.strafeEncoder.getCurrentPosition();
            drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, forwardTarget,startStrafe,0,0,null,0,3000);
        }
        if (gamepad1.dpad_right) {
            drive.setHeading(0);
            long startStrafe = drive.strafeEncoder.getCurrentPosition();
            long strafeTarget = (long) (startStrafe + drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
            long startForward = drive.forwardEncoder.getCurrentPosition();
            drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, strafeTarget,startForward,0,0,null,0,3000);
        }
        if (gamepad1.dpad_right) {
            drive.setHeading(0);
            long startStrafe = drive.strafeEncoder.getCurrentPosition();
            long strafeTarget = (long) (startStrafe - drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
            long startForward = drive.forwardEncoder.getCurrentPosition();
            drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, strafeTarget,startForward,0,0,null,0,3000);
        }
    }

    public void testMoveTo() {
        //TBD  Set up some button presses to move the robot to some X,Y values?
    }

    public void goForADriveCMs() {
        drive.moveCm(testDistance, 90);
        drive.moveCm(testDistance, 180);
        drive.moveCm(testDistance, 270);
        drive.moveCm(testDistance, 0);

        drive.moveCm(testDistance, 45);
        drive.moveCm(testDistance, 135);
        drive.moveCm(testDistance, 225);
        drive.moveCm(testDistance, 315);
    }

    public void goForADriveTarget() {
        long startForward = drive.forwardEncoder.getCurrentPosition();
        long forwardTarget = (long) (startForward + drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
        long startStrafe = drive.strafeEncoder.getCurrentPosition();
        long strafeTarget = (long) (startStrafe + drive.TICS_PER_CM_STRAIGHT_ENCODER*testDistance);
        drive.setHeading(0);

        drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, forwardTarget,startStrafe,0,0,null,0,3000);
        drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, strafeTarget,forwardTarget,0,0,null,0,3000);
        drive.straightHoldingStrafeEncoder(drive.MAX_VELOCITY, startForward,strafeTarget,0,0,null,0,3000);
        drive.strafeHoldingStraightEncoder(drive.MAX_VELOCITY, startStrafe,startForward,0,0,null,0,3000);
    }

    public void goForADriveAngleTargets() {
        // TBD: Implement a loop using MoveTo()
    }

    public void brakeTestForward () {
        drive.setHeading(0);
        drive.moveCm(testVelocity   , testDistance  , 0, 0,testVelocity);
        drive.setMotorsBrake();
        drive.stopMotors();
        while (drive.forwardEncoder.getVelocity()> 0) {
            telemetry.addData("Velocity", drive.forwardEncoder.getVelocity());
            telemetry.addData("Encoder", drive.forwardEncoder.getCurrentPosition());
        }
    }
    public void brakeTestRight () {
        drive.setHeading(0);
        drive.moveCm(testVelocity   , testDistance  , 270, 0,testVelocity);
        drive.setMotorsBrake();
        drive.stopMotors();
        while (drive.forwardEncoder.getVelocity()> 0) {
            telemetry.addData("Velocity", drive.strafeEncoder.getVelocity());
            telemetry.addData("Encoder", drive.strafeEncoder.getCurrentPosition());
        }
    }
}
