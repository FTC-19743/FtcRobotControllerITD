package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import com.acmerobotics.dashboard.config.Config;

import com.acmerobotics.dashboard.FtcDashboard;

@Config

@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    TeamGamepad gamepad;




    int delay = 0;
    boolean cycle = true;
    int cycleDelay = 0;
    boolean ascent = false;
    int blocks = 1;
    int specimen = 1;


    public void initializeRobot() {
        telemetry.addLine("Initializing Robot");
        teamUtil.telemetry.update();
        telemetry.update();
        robot = new Robot();
        robot.initialize();
    }



    @Override
    public void runOpMode() {
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        teamUtil.alliance = teamUtil.Alliance.RED;
        //Calibrate
        while (!gamepad.wasAPressed()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Check Robot and THEN");
            teamUtil.telemetry.addLine("Press X on Game Pad 1 to CALIBRATE");
            teamUtil.telemetry.update();
        }
        initializeRobot();
        robot.calibrate();
        teamUtil.robot = robot;

        while (!gamepad.wasAPressed()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Press Bumpers To Change Alliance");
            if (gamepad.wasRightBumperPressed() || gamepad.wasLeftBumperPressed()) {
                if (teamUtil.alliance == teamUtil.Alliance.BLUE) {
                    teamUtil.alliance = teamUtil.Alliance.RED;
                } else {
                    teamUtil.alliance = teamUtil.Alliance.BLUE;
                }
            }
            teamUtil.telemetry.update();
        }

        while (!gamepad.wasAPressed()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);

            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Press Bumpers To Change SIDE");
            if (gamepad.wasRightBumperPressed() || gamepad.wasLeftBumperPressed()) {
                if (teamUtil.SIDE == teamUtil.SIDE.BASKET) {
                    teamUtil.SIDE = teamUtil.SIDE.OBSERVATION;
                } else {
                    teamUtil.SIDE = teamUtil.SIDE.BASKET;

                }
            }
            teamUtil.telemetry.update();
        }

        while (!gamepad.wasAPressed()) {
            gamepad.loop();
            if (gamepad.wasUpPressed()) {
                specimen++;
                if (specimen > 3) {
                    specimen = 0;
                }
            }
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Delay: " + delay);
            teamUtil.telemetry.addLine("Press Right to Increase Delay");
            teamUtil.telemetry.addLine("Press Left to Decrease Delay");
            if(gamepad.wasLeftPressed()){
                delay--;
            }
            if(gamepad.wasRightPressed()){
                delay++;
            }
            teamUtil.telemetry.addLine("Press X to Move On");
        }
        if (teamUtil.SIDE == teamUtil.Side.BASKET) {
            //Blocks
            while (!gamepad.wasAPressed()) {
                gamepad.loop();
                if (gamepad.wasUpPressed()) {
                    blocks++;
                    if (blocks > 3) {
                        blocks = 1;
                    }
                }
                teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
                teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
                teamUtil.telemetry.addLine("Blocks: " + blocks);
                teamUtil.telemetry.addLine("Delay: " + delay);
                teamUtil.telemetry.addLine("Press Up to Add Block");
                teamUtil.telemetry.addLine("Press X to Move On");


                teamUtil.telemetry.update();
            }

            while (!gamepad.wasAPressed()) {
                gamepad.loop();
                if (gamepad.wasUpPressed()) {
                    if (ascent) {
                        ascent = false;
                    } else {
                        ascent = true;
                    }
                }
                teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
                teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
                teamUtil.telemetry.addLine("Ascent: " + ascent);
                teamUtil.telemetry.addLine("Blocks: " + blocks);
                teamUtil.telemetry.addLine("Delay: " + delay);
                teamUtil.telemetry.addLine("Press Up Change Ascent");
                teamUtil.telemetry.addLine("Press X to Move On");
                teamUtil.telemetry.update();
            }

        } else {
            //Specimen
            while (!gamepad.wasAPressed()) {
                gamepad.loop();
                if (gamepad.wasUpPressed()) {
                    specimen++;
                    if (specimen > 4) {
                        specimen = 0;
                    }
                }
                teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
                teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
                teamUtil.telemetry.addLine("Specimen: " + specimen);
                teamUtil.telemetry.addLine("Press Up to Add Specimen");
                teamUtil.telemetry.addLine("Delay: " + delay);
                teamUtil.telemetry.addLine("Press X to Move On");


                teamUtil.telemetry.update();
            }
        }

        while (!gamepad.wasAPressed()) {
            gamepad.loop();
            teamUtil.telemetry.addLine("Press Up To Extend Hang");
            teamUtil.telemetry.addLine("Press Down To Stow Hang");

            if(gamepad.wasUpPressed()){
                robot.hang.extendHang();
            }
            if(gamepad.wasDownPressed()){
                robot.hang.stowHang();
            }

            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Specimen: " + specimen);
            teamUtil.telemetry.addLine("Press Up to Add Specimen");
            teamUtil.telemetry.addLine("Delay: " + delay);
            teamUtil.telemetry.addLine("Press X to Move On");

            teamUtil.telemetry.update();
        }

        if (teamUtil.SIDE == teamUtil.SIDE.BASKET) { // get camera running but only if we will use it
            robot.initCV(false); // no live stream enabled means better FPS
        }

        while (!opModeIsActive()) {
            telemetry.addLine("Ready to Go!");
            teamUtil.telemetry.addLine("Alliance: " + teamUtil.alliance);
            teamUtil.telemetry.addLine("Side: " + teamUtil.SIDE);
            teamUtil.telemetry.addLine("Delay: " + delay);
            if (teamUtil.SIDE == teamUtil.Side.BASKET) {
                telemetry.addLine("Blocks: " + blocks);
                telemetry.addLine("Ascent: " + ascent);
            } else {
                teamUtil.telemetry.addLine("Specimen: " + specimen);
            }
            telemetry.update();
        }

        waitForStart();
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        long startTime = System.currentTimeMillis();
        teamUtil.pause(delay);
        if(teamUtil.SIDE == teamUtil.Side.BASKET){
            //robot.autoV1Bucket(blocks, ascent);
        }else{
            robot.autoV2Specimen(specimen);
        }
        long endTime = System.currentTimeMillis();
        long elapsedTime = endTime - startTime;
        teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);

        while (opModeIsActive()) {}

        teamUtil.justRanAuto = true; // avoid recalibration at start of teleop
    }
}


