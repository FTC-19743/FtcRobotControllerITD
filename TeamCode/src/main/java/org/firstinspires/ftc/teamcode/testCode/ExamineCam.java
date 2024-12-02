package org.firstinspires.ftc.teamcode.testCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.FocusControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.WhiteBalanceControl;
import org.firstinspires.ftc.robotcore.internal.camera.delegating.CachingExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.api.UvcApiExposureControl;
import org.firstinspires.ftc.robotcore.internal.camera.libuvc.nativeobject.UvcDeviceHandle;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.lang.reflect.Field;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "Examine Cam", group = "Test Code")
public class ExamineCam extends LinearOpMode
{
    TeamGamepad gp1 = new TeamGamepad();
    int normalExposure = 1;

    public void runOpMode()
    {
        teamUtil.init(this);
        gp1.initilize(true);

        VisionPortal portal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "arducam"));

        UvcDeviceHandle uvcDeviceHandle = null;

        while (!isStopRequested())
        {
            if (portal.getCameraState() == VisionPortal.CameraState.STREAMING)
            {
                CachingExposureControl cachingExposureControl = (CachingExposureControl) portal.getCameraControl(ExposureControl.class);
                if (cachingExposureControl == null)
                {
                    throw new RuntimeException("Failed to get exposure control");
                }

                try
                {
                    Field f = CachingExposureControl.class.getDeclaredField("delegatedExposureControl");
                    f.setAccessible(true);
                    UvcApiExposureControl uvcApiExposureControl = (UvcApiExposureControl) f.get(cachingExposureControl);

                    Field f2 = UvcApiExposureControl.class.getDeclaredField("uvcDeviceHandle");
                    f2.setAccessible(true);
                    uvcDeviceHandle = (UvcDeviceHandle) f2.get(uvcApiExposureControl);
                }
                catch (Exception e)
                {
                    throw new RuntimeException("Failed to reflect");
                }

                uvcDeviceHandle.setExposureMode(ExposureControl.Mode.Manual);
                uvcDeviceHandle.setAePriority(false);

                break;
            }
            telemetry.addData("Camera State", portal.getCameraState());
            telemetry.update();
        }
        configureCam(portal, false, false, 1, 127, true,0,true,0);
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);

        while (!isStopRequested())
        {
            gp1.loop();
            long requestedExposure = (uvcDeviceHandle.getMinExposure() + uvcDeviceHandle.getMaxExposure()) / 2;
            int requestedGain = (uvcDeviceHandle.getMinGain() + uvcDeviceHandle.getMaxGain()) / 2;

            exposureControl.setExposure(normalExposure, TimeUnit.MILLISECONDS);
            //uvcDeviceHandle.setExposure(requestedExposure);

            telemetry.addLine("Exposure Range (ns): "+ uvcDeviceHandle.getMinExposure()+"-"+uvcDeviceHandle.getMaxExposure());
            telemetry.addData("Current Exposure (ns)", uvcDeviceHandle.getExposure());
            telemetry.addData("Current Exposure (ms)", uvcDeviceHandle.getExposure()/1000000);
            telemetry.addData("Requested Exposure (ns)", requestedExposure);
            telemetry.addData("Requested Exposure (normal)", normalExposure);

            uvcDeviceHandle.setGain(requestedGain);

            telemetry.addLine("Gain Range : "+ uvcDeviceHandle.getMinGain()+"-"+uvcDeviceHandle.getMaxGain());
            telemetry.addData("Current Gain ", uvcDeviceHandle.getGain());
            telemetry.addData("Requested Gain ", requestedGain);
            telemetry.update();

            if (gp1.wasUpPressed()) normalExposure++;
            if (gp1.wasDownPressed()) normalExposure--;
        }
    }

    public void configureCam (VisionPortal portal, boolean autoExposure, boolean aePriority, long exposure, int gain, boolean autoWB, int wb, boolean aFocus, int focalLength) {
        ExposureControl exposureControl = portal.getCameraControl(ExposureControl.class);
        GainControl gainControl = portal.getCameraControl(GainControl.class);
        WhiteBalanceControl wbControl = portal.getCameraControl(WhiteBalanceControl.class);
        FocusControl focusControl = portal.getCameraControl(FocusControl.class);

        if (autoExposure) {
            if (exposureControl.setMode(ExposureControl.Mode.AperturePriority)) {
                teamUtil.log("Set WebCam to Auto Exposure");
            } else {
                teamUtil.log("FAILED to set webcam to auto exposure");
            }
        } else {
            if (exposureControl.setMode(ExposureControl.Mode.Manual)) {
                teamUtil.log("Set WebCam to Manual Exposure");
            } else {
                teamUtil.log("FAILED to set webcam to Manual exposure");
            }
            if (exposureControl.setExposure(exposure, TimeUnit.MILLISECONDS)) {
                teamUtil.log("Set WebCam Exposure to "+ exposure);
                teamUtil.log("WebCam Exposure now "+ exposureControl.getExposure(TimeUnit.MILLISECONDS));

            } else {
                teamUtil.log("FAILED to set WebCam Exposure");
            }
            if (gainControl.setGain(gain)) {
                teamUtil.log("Set WebCam Gain to "+ gain);
            } else {
                teamUtil.log("FAILED to set WebCam Gain");
            }
        }

        if (exposureControl.setAePriority(aePriority)) {
            teamUtil.log("Set WebCam aePriority to "+ aePriority);
        } else {
            teamUtil.log("FAILED to set WebCam aePriority");
        }

        if (autoWB) {
            if (wbControl.setMode(WhiteBalanceControl.Mode.AUTO)) {
                teamUtil.log("Set WebCam to Auto White Balance");
            } else {
                teamUtil.log("FAILED to set webcam to auto White Balance");
            }
        } else {
            if (wbControl.setMode(WhiteBalanceControl.Mode.MANUAL)) {
                teamUtil.log("Set WebCam to Manual White Balance");
            } else {
                teamUtil.log("FAILED to set webcam to Manual White Balance");
            }
            if (wbControl.setWhiteBalanceTemperature(wb)) {
                teamUtil.log("Set WebCam WB to "+ wb);
            } else {
                teamUtil.log("FAILED to set WebCam WB");
            }
        }
        if (aFocus) {
            if (focusControl.setMode(FocusControl.Mode.ContinuousAuto)) {
                teamUtil.log("Set WebCam to Continuous Auto Focus");
            } else {
                teamUtil.log("FAILED to set webcam to Continuous Auto Focus");
            }
        } else {
            if (focusControl.setMode(FocusControl.Mode.Fixed)) {
                teamUtil.log("Set WebCam to Fixed Focus");
            } else {
                teamUtil.log("FAILED to set webcam to Fixed Focus");
            }
            if (focusControl.setFocusLength(focalLength)) {
                teamUtil.log("Set WebCam focal length to "+ focalLength);
            } else {
                teamUtil.log("FAILED to set WebCam focal length");
            }
        }
    }
}

