package org.firstinspires.ftc.teamcode.testCode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
@TeleOp(name = "Test Drive", group = "Test Code")
public class TestDrive extends LinearOpMode {
    public static int botX = 72;
    public static int botY = 72;

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) {
                continue;
            }

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        //FtcDashboard.setDrawDefaultField(false); // enable to eliminate field drawing


        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay().drawImage("/images/BatBot.jpg", botX, botY, 18, 18);
            //packet.fieldOverlay().drawImage("/dash/powerplay.png", 0, 0, 144, 144);
            logGamepad(telemetry, gamepad1, "gamepad1");
            logGamepad(telemetry, gamepad2, "gamepad2");
            dashboard.sendTelemetryPacket(packet);
            sleep(20);
        }
    }

}
