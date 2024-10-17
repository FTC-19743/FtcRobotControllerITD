package org.firstinspires.ftc.teamcode.testCode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.PtzControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.teamcode.assemblies.Robot;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Webcam Test ", group = "Test Code")
public class webcamTest extends LinearOpMode {


    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;
    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);




    public void runOpMode() {
        teamUtil.init(this);

        VisionPortal portal;
        telemetry.addLine("Ready to start");

        CameraName cam = (CameraName)hardwareMap.get(WebcamName.class, "logitechhd");
        CameraCharacteristics chars = cam.getCameraCharacteristics();
        teamUtil.log(cam.toString());
        teamUtil.log("WebCam: "+(cam.isWebcam() ? "true" : "false"));
        teamUtil.log("Unknown: "+(cam.isUnknown() ? "true" : "false"));
        teamUtil.log(chars.toString());

        teamUtil.log("Setting up VisionPortal");
        VisionPortal.Builder armBuilder = new VisionPortal.Builder();
        armBuilder.setCamera(cam);
        armBuilder.enableLiveView(true);
        // Can also set resolution and stream format if we want to optimize resource usage.
        //armBuilder.setStreamFormat(TBD);
        armBuilder.setCameraResolution(arduSize);
        portal = armBuilder.build();

        // Wait for the camera to be open
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (portal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        teamUtil.log("---------------------------------------------------------------");
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        teamUtil.log("Exposure Supported: " + exposureControl.isExposureSupported());
        teamUtil.log("Exposure Modes: A:" + exposureControl.isModeSupported(ExposureControl.Mode.Auto) +
                " M:" + exposureControl.isModeSupported(ExposureControl.Mode.Manual) +
                " AP:" + exposureControl.isModeSupported(ExposureControl.Mode.AperturePriority) +
                " CA:" + exposureControl.isModeSupported(ExposureControl.Mode.ContinuousAuto) +
                " SP:" + exposureControl.isModeSupported(ExposureControl.Mode.ShutterPriority) +
                " U:" + exposureControl.isModeSupported(ExposureControl.Mode.Unknown));
        teamUtil.log("Exposure Mode: " + exposureControl.getMode());
        teamUtil.log("AE Priority: " + exposureControl.getAePriority());
        teamUtil.log("Exposure Range: " +
                (int)(exposureControl.getMinExposure(TimeUnit.MILLISECONDS) + 1) + "-" +
                (int)exposureControl.getMaxExposure(TimeUnit.MILLISECONDS));

        teamUtil.log("---------------------------------------------------------------");

        GainControl gainControl = portal.getCameraControl(GainControl.class);
        teamUtil.log("Gain Range: " + gainControl.getMinGain() + "-" + gainControl.getMaxGain());
        teamUtil.log("Gain: " + gainControl.getGain());
        teamUtil.log("Gain Supported: " + gainControl.setGain(127));

        teamUtil.log("---------------------------------------------------------------");
        WhiteBalanceControl wbControl = portal.getCameraControl(WhiteBalanceControl.class);
        teamUtil.log("White Balance Mode: " + wbControl.getMode());
        teamUtil.log("White Balance Modes: A:" + wbControl.setMode(WhiteBalanceControl.Mode.AUTO) +
                " M:" + wbControl.setMode(WhiteBalanceControl.Mode.AUTO));
        teamUtil.log("Temp Range: " + wbControl.getMinWhiteBalanceTemperature() + "-" + wbControl.getMaxWhiteBalanceTemperature());
        teamUtil.log("Temp: " + wbControl.getWhiteBalanceTemperature());
        teamUtil.log("Temp Supported: " + wbControl.setWhiteBalanceTemperature(4000));

        teamUtil.log("---------------------------------------------------------------");
        FocusControl focusControl = portal.getCameraControl(FocusControl.class);
        teamUtil.log("Focus Supported: " + focusControl.isFocusLengthSupported());
        teamUtil.log("Focus Modes: A:" + focusControl.isModeSupported(FocusControl.Mode.Auto) +
                " CA:" + focusControl.isModeSupported(FocusControl.Mode.ContinuousAuto) +
                " F:" + focusControl.isModeSupported(FocusControl.Mode.Fixed) +
                " I:" + focusControl.isModeSupported(FocusControl.Mode.Infinity) +
                " M:" + focusControl.isModeSupported(FocusControl.Mode.Macro) +
                " U:" + focusControl.isModeSupported(FocusControl.Mode.Unknown));
        teamUtil.log("Focus Mode: " + focusControl.getMode());
        teamUtil.log("Focus Range: " + focusControl.getMinFocusLength() + "-" + focusControl.getMaxFocusLength());

        teamUtil.log("---------------------------------------------------------------");
        PtzControl ptzControl = portal.getCameraControl(PtzControl.class);
        teamUtil.log("Zoom Range: " + ptzControl.getMinZoom() + "-" + ptzControl.getMaxZoom());
        teamUtil.log("Zoom: " + ptzControl.getZoom());
        teamUtil.log("Zoom Supported: " + ptzControl.setZoom(200));




        telemetry.addLine("Ready to start");
        telemetry.addLine("ALLIANCE : "+ teamUtil.alliance);
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){}
    }
}
