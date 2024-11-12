package org.firstinspires.ftc.teamcode.assemblies;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name = "Auto", group = "LinearOpMode")
public class Auto extends LinearOpMode {
    Robot robot;
    TeamGamepad gamepad;

    int delay = 0;
    boolean cycle = true;
    int cycleDelay = 0;



    public void initializeRobot(){
        telemetry.addLine("Initializing Robot");
        teamUtil.telemetry.update();
        telemetry.update();
        robot = new Robot();
        robot.initialize();
    }
    @Override
    public void runOpMode(){
        teamUtil.init(this);
        gamepad = new TeamGamepad();
        gamepad.initilize(true);
        teamUtil.alliance = teamUtil.Alliance.RED;
        //Calibrate
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            teamUtil.telemetry.addLine("Check Robot and THEN");
            teamUtil.telemetry.addLine("Press X on Game Pad 1 to CALIBRATE");
            teamUtil.telemetry.update();
        }
        initializeRobot();
        robot.calibrate();
        teamUtil.robot = robot;


        //Blocks
        int blocks = 1;
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasUpPressed()){
                blocks++;
                if(blocks>3){
                    blocks=1;
                }
            }
            teamUtil.telemetry.addLine("Blocks: " + blocks);
            teamUtil.telemetry.addLine("Press Up to Add Block");
            teamUtil.telemetry.addLine("Press X to Move On");



            teamUtil.telemetry.update();
        }
        boolean ascent = false;
        while(!gamepad.wasAPressed()){
            gamepad.loop();
            if(gamepad.wasUpPressed()){
                if(ascent){
                    ascent = false;
                }else{
                    ascent=true;
                }
            }
            teamUtil.telemetry.addLine("Ascent: " + ascent);
            teamUtil.telemetry.addLine("Blocks: " + blocks);

            teamUtil.telemetry.addLine("Press Up Change Ascent");
            teamUtil.telemetry.addLine("Press X to Move On");
            teamUtil.telemetry.update();
        }

        robot.initCV(false); // no live stream enabled means better FPS




        while(!opModeIsActive()){

            telemetry.addLine("Ready to Go!");
            telemetry.addLine("Blocks: " + blocks);
            teamUtil.telemetry.addLine("Ascent: " + ascent);

            telemetry.addLine("Alliance: "+teamUtil.alliance.toString());
            telemetry.update();



            /*
            if(path == 1 && teamUtil.alliance == teamUtil.alliance.RED){
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_1);
            } else if (path == 2 && teamUtil.alliance == teamUtil.alliance.RED) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_2);
            } else if (path == 3 && teamUtil.alliance == teamUtil.alliance.RED) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.RED_PATH_3);
            } else if (path == 1 && teamUtil.alliance == teamUtil.alliance.BLUE) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_1);
            } else if (path == 2 && teamUtil.alliance == teamUtil.alliance.BLUE) {
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_2);
            } else{
                teamUtil.theBlinkin.setSignal(Blinkin.Signals.BLUE_PATH_3);
            }

             */
        }

        waitForStart();
        //teamUtil.theBlinkin.setSignal(Blinkin.Signals.OFF);
        long startTime = System.currentTimeMillis();
        robot.autoV1Bucket(blocks,ascent);
        long endTime = System.currentTimeMillis();
        long elapsedTime = endTime-startTime;
        teamUtil.log("Elapsed Auto Time Without Wait At End: " + elapsedTime);

        while(opModeIsActive()){
        }

        teamUtil.justRanAuto = true; // avoid recalibration at start of teleop

    }
}

