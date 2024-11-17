package org.firstinspires.ftc.teamcode.libs;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.assemblies.Robot;

public class teamUtil {

    public static LinearOpMode theOpMode;
    public static boolean inInitialization = true;
    public static Telemetry telemetry;
    public static boolean justRanAuto = false;
    public static boolean justRanCalibrateRobot = false;

    public static Robot robot;

    public enum Alliance {RED, BLUE}
    public static Alliance alliance = Alliance.RED;
    public enum Side {BASKET, OBSERVATION}
    public static Side SIDE = Side.BASKET;

    public static Blinkin theBlinkin = null;

    public static boolean finishedGoToLoad = true;

    public static boolean LEFT = true; // Don't use this anymore!

    public static long startTime;


    public static void init (LinearOpMode opMode) {
        theOpMode = opMode;
        telemetry = theOpMode.telemetry;
        inInitialization = true;
        theBlinkin = new Blinkin(opMode.hardwareMap, opMode.telemetry);
        theBlinkin.init();
    }



    // Wait for the specified milliseconds
    public static void pause(long sleepTime) {
        long wakeupTime= System.currentTimeMillis()+sleepTime;
        while(System.currentTimeMillis()< wakeupTime){
            theOpMode.idle();
        }
    }

    // log something so we can filter out the FTC robot log info
    public static void log(String logString) {
        RobotLog.d("19743LOG:T" + Thread.currentThread().getId()+" "+ Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    // A helper method to see if a long running operation should continue.
    // Call this in your loops
    public static boolean keepGoing(long timeOutTime) {
        String s=Long.toString(System.currentTimeMillis());

        //log(s);
        return ((inInitialization || theOpMode.opModeIsActive()) && (System.currentTimeMillis() < timeOutTime));
    }
}
