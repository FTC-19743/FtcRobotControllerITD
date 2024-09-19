package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.assemblies.BasicDrive;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.SynchronousQueue;

@Config
@TeleOp(name = "SpecimenTransport Test", group = "Test Code")
public class SpecimenTransportTest extends LinearOpMode {

    public static float wristGrab= 0.84f;
    public static float wristDeliver = 0.17f;

    public static float armGrab= 0.0f;
    public static float armDeliver= 0.71f;


    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        Servo arm = hardwareMap.get(Servo.class,"arm");
        Servo wrist = hardwareMap.get(Servo.class,"wrist");

        teamUtil.init(this);


        gp1.initilize(true); // Game Pads can be plugged into the computer
        gp2.initilize(false);

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            gp1.loop();
            gp2.loop();


            // Left bumper toggles Alliance
            if (gp1.wasLeftPressed()) {
                arm.setPosition(armDeliver);
            }
            if (gp1.wasRightPressed()) {
                arm.setPosition(armGrab);
            }
            if (gp1.wasUpPressed()) {
                wrist.setPosition(wristDeliver);
            }
            if (gp1.wasDownPressed()) {
                wrist.setPosition(wristGrab);
            }
            if (gp1.wasYPressed()){
                arm.setPosition(armDeliver);
                teamUtil.pause(500);
                wrist.setPosition(wristDeliver);
            }
            if (gp1.wasXPressed()){
                arm.setPosition(armGrab);
                wrist.setPosition(wristGrab);
            }





            // Drawing stuff on the field
            TelemetryPacket packet = new TelemetryPacket();
            //packet.fieldOverlay().drawImage("/images/BatBot.jpg", botX, botY, 18, 18);
            //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            dashboard.sendTelemetryPacket(packet);



            telemetry.update();
        }
    }


}
