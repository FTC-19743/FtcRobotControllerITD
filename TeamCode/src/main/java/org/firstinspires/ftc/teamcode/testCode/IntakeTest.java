package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
@TeleOp(name = "Intake Test", group = "Test Code")
public class IntakeTest extends LinearOpMode {

    public static float clawGrab= 0.84f;
    public static float clawExpand = 0.17f;

    //public static float armDeliver= 0.06f;
    //public static float armGrab= 0.625f;


    private TeamGamepad gp1 = new TeamGamepad();
    private TeamGamepad gp2 = new TeamGamepad();

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard
        Servo claw = hardwareMap.get(Servo.class,"claw");
        //Servo wrist = hardwareMap.get(Servo.class,"wrist");

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
                claw.setPosition(clawExpand);
            }
            if (gp1.wasRightPressed()) {
                claw.setPosition(clawGrab);
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
