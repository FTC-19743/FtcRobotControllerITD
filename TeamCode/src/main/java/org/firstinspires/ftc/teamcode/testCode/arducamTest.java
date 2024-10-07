/*
 * Copyright (c) 2023 FIRST
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.testCode;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.libs.OpenCVSampleDetector;
import org.firstinspires.ftc.teamcode.libs.TeamGamepad;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;

/*
 * This OpMode helps calibrate a webcam or RC phone camera, useful for AprilTag pose estimation
 * with the FTC VisionPortal.   It captures a camera frame (image) and stores it on the Robot Controller
 * (Control Hub or RC phone), with each press of the gamepad button X (or Square).
 * Full calibration instructions are here:
 *
 *  https://ftc-docs.firstinspires.org/camera-calibration
 *
 * In Android Studio, copy this class into your "teamcode" folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * In OnBot Java, use "Add File" to add this OpMode from the list of Samples.
 */
@Config

@TeleOp(name = "Arducam Test Code", group = "Test Code")

public class arducamTest extends LinearOpMode
{
    /*
     * EDIT THESE PARAMETERS AS NEEDED
     */
    final boolean USING_WEBCAM = true;
    final BuiltinCameraDirection INTERNAL_CAM_DIR = BuiltinCameraDirection.BACK;
    final int ARDU_RESOLUTION_WIDTH = 640;
    final int ARDU_RESOLUTION_HEIGHT = 480;

    Size arduSize = new Size(ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT);

    // Internal state
    boolean lastX;
    int frameCount;
    long capReqTime;

    public static int blueLowH1=0;
    public static int blueLowS1=0;
    public static int blueLowV1=0;

    public static int blueHighH1=0;
    public static int blueHighS1=0;
    public static int blueHighV1=0;

    public static int blueErosionFactor1=0;






    @Override
    public void runOpMode()
    {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry()); // write telemetry to Driver Station and Dashboard

        teamUtil.init(this);
        TeamGamepad gp1 = new TeamGamepad();
        gp1.initilize(true);

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
        //armBuilder.setLiveViewContainerId(visionPortalViewIDs[0]);
        //if (!enableLiveView) {
            armBuilder.enableLiveView(true);
        //}
        // Can also set resolution and stream format if we want to optimize resource usage.
        armBuilder.setCameraResolution(arduSize);
        //armBuilder.setStreamFormat(TBD);

        armBuilder.addProcessor(sampleDetector);

        arduPortal = armBuilder.build();
        //stopStreaming(arduPortal);
        //arduPortal.setProcessorEnabled(findPixelProcesser,false);


        while (!opModeIsActive()) {}

        int currentTargetColor=1;
        sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);
        while (!isStopRequested())
        {

            gp1.loop();
            /* TEST CODE FOR CHANGING VALUES LIVE
            sampleDetector.blueLowH = blueLowH1;
            sampleDetector.blueLowS = blueLowS1;
            sampleDetector.blueLowV = blueLowV1;

            sampleDetector.blueHighH = blueHighH1;
            sampleDetector.blueHighS = blueHighS1;
            sampleDetector.blueHighV = blueHighV1;

            sampleDetector.blueErosionFactor = blueErosionFactor1;

            sampleDetector.blueLowHSV = new Scalar(blueLowH1,blueLowS1,blueLowV1);
            sampleDetector.blueHighHSV = new Scalar(blueHighH1,blueHighS1,blueHighV1);

             */

            if (gp1.wasXPressed())
            {
                arduPortal.saveNextFrameRaw(String.format(Locale.US, "CameraFrameCapture-%06d", frameCount++));
                capReqTime = System.currentTimeMillis();
            }
            if (gp1.wasYPressed()){
                sampleDetector.nextView();
            }
            if(gp1.wasUpPressed()){
                currentTargetColor+=1;
                if(currentTargetColor>3){
                    currentTargetColor=1;
                    sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.YELLOW);

                }
                else if(currentTargetColor==2){
                    sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.RED);
                }else{
                    sampleDetector.setTargetColor(OpenCVSampleDetector.TargetColor.BLUE);
                }
            }


            //telemetry.addLine("Blue Erosion Factor"+ blueErosionFactor1);
            //telemetry.addLine("Blue Erosion Factor"+ sampleDetector.blueErosionFactor);

            telemetry.addLine("Current Color: "+ sampleDetector.targetColor);
            telemetry.addLine("######## Camera Capture Utility ########");
            telemetry.addLine(String.format(Locale.US, " > Resolution: %dx%d", ARDU_RESOLUTION_WIDTH, ARDU_RESOLUTION_HEIGHT));
            telemetry.addLine(" > Press X (or Square) to capture a frame");
            telemetry.addData(" > Camera Status", arduPortal.getCameraState());
            telemetry.addLine("Found One : "+sampleDetector.foundOne);
            telemetry.addLine("Rect Angle : "+sampleDetector.rectAngle);
            telemetry.addLine("Rect Center X : "+sampleDetector.rectCenterXOffset);
            telemetry.addLine("Rect Center Y : "+sampleDetector.rectCenterYOffset);




            if (capReqTime != 0)
            {
                telemetry.addLine("\nCaptured Frame!");
            }

            if (capReqTime != 0 && System.currentTimeMillis() - capReqTime > 1000)
            {
                capReqTime = 0;
            }

            telemetry.update();
        }
    }
}
