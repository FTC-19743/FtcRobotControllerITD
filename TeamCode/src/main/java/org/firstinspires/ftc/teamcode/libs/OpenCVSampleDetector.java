package org.firstinspires.ftc.teamcode.libs;

import
        android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;

public class OpenCVSampleDetector extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Scalar BLACK = new Scalar(0, 0, 0);
    Scalar PASTEL_GREEN  = new Scalar(204, 255, 204);
    Scalar PASTEL_RED = new Scalar(204, 204, 255);
    Scalar PASTEL_PURPLE = new Scalar(255, 204, 204);

    public final int WIDTH = 640;
    public final int HEIGHT = 480;

    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat erodedMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchyMat = new Mat();

    private int blurFactor = 10;
    private Size blurFactorSize = new Size(blurFactor,blurFactor);

    private int yellowLowH = 15, yellowLowS = 100, yellowLowV = 100;
    private int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    private int yellowErosionFactor = 20;
    Scalar yellowLowHSV = new Scalar(yellowLowH, yellowLowS, yellowLowV); // lower bound HSV for yellow
    Scalar yellowHighHSV = new Scalar(yellowHighH, yellowHighS, yellowHighV); // higher bound HSV for yellow
    Mat yellowErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * yellowErosionFactor + 1, 2 * yellowErosionFactor + 1),
            new Point(yellowErosionFactor, yellowErosionFactor));
    // TODO Repeat above section for Red
    public int blueLowH = 100, blueLowS = 100, blueLowV = 50;
    public int blueHighH = 120, blueHighS = 255, blueHighV = 200;
    public int blueErosionFactor = 15;
    public Scalar blueLowHSV = new Scalar(blueLowH, blueLowS, blueLowV); // lower bound HSV for yellow
    public Scalar blueHighHSV = new Scalar(blueHighH, blueHighS, blueHighV); // higher bound HSV for yellow
    Mat blueErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * blueErosionFactor + 1, 2 * blueErosionFactor + 1),
            new Point(blueErosionFactor, blueErosionFactor));


    public enum TargetColor {
        YELLOW,
        RED,
        BLUE
    }
    public TargetColor targetColor = TargetColor.YELLOW;
    public void setTargetColor(TargetColor newTargetColor){
        targetColor = newTargetColor;
    }

    public AtomicBoolean foundOne = new AtomicBoolean(false);
    // TODO: Data about the located Sample

    public boolean viewingPipeline = false;
    enum Stage {
        RAW_IMAGE,
        HSV,
        BLURRED,
        THRESHOLD,
        ERODED,
        EDGES,
        FINAL
    }
    private Stage stageToRenderToViewport = Stage.FINAL;
    private Stage[] stages = Stage.values();

    public OpenCVSampleDetector() {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVSampleDetector processor");
        teamUtil.log("Initializing OpenCVSampleDetector processor - FINISHED");
    }
    public void outputTelemetry () {
        telemetry.addLine("Found One: " + foundOne.get() + "TBD");
    }

    public void nextView() {

        int currentStageNum = stageToRenderToViewport.ordinal();
        int nextStageNum = currentStageNum + 1;
        if (nextStageNum >= stages.length) {
            nextStageNum = 0;
        }
        stageToRenderToViewport = stages[nextStageNum];
    }

    List<MatOfPoint> contours = new ArrayList<>();
    double largestArea;

    public void reset() {
        foundOne.set(false);
    }
    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        boolean details = false;
        if (details) teamUtil.log("Sample Detector: Process Frame");

        Rect cropRect = new  Rect(0,0,frame.width(), frame.height());

        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV

        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise

        //Imgproc.rectangle(blurredMat, cropRect, blackColor,2); // black frame to help with erosion and boundaries. TODO: Why was this removed? Does it not affect Erosion?

        switch (targetColor) {
            case YELLOW:
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, yellowErosionElement);
                break;
            case RED:
                teamUtil.log("RED not implemented");
                break;
            case BLUE:
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, blueErosionElement);
                break;
        }

        Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges

        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around white areas

        if (!contours.isEmpty()) {
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            RotatedRect[] boundRect = new RotatedRect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
            }
            return boundRect; // return array of rotated rectangles we found
        }  else {
            if (details) teamUtil.log("No Detections");
        }
        return null;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // draw rectangles around the objects we found
        if (viewingPipeline) {
            Bitmap bmp = Bitmap.createBitmap(HSVMat.cols(), HSVMat.rows(), Bitmap.Config.ARGB_8888);
            switch (stageToRenderToViewport) {
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case ERODED: { Utils.matToBitmap(erodedMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edgesMat, bmp); break;}
                default: {}
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(WIDTH*scaleBmpPxToCanvasPx), (int)(HEIGHT*scaleBmpPxToCanvasPx), false);
            canvas.drawBitmap(resizedBitmap, 0,0,null);
        }

        if (userContext != null) {
            // TODO Annotate with centers of circles and maybe rotation vectors
            RotatedRect[] boundRect = (RotatedRect[]) userContext;
            Paint rectPaint = new Paint();
            rectPaint.setColor(Color.RED);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 4);
            canvas.drawCircle(320, 240, 60,rectPaint);
            double smallestDistFromCenter=0;
            double xDist;
            double yDist;
            for (int i = 0; i < boundRect.length; i++) {
                //Old Normal Circle Drawing
                //canvas.drawCircle((float)boundRect[i].center.x*scaleBmpPxToCanvasPx, (float)boundRect[i].center.y*scaleBmpPxToCanvasPx, 3,rectPaint);
                teamUtil.log("Center (X) " + (float)boundRect[i].center.x*scaleBmpPxToCanvasPx);
                teamUtil.log("Height" + (float)boundRect[i].size.height);
                teamUtil.log("Height" + (float)boundRect[i].size.width);
                teamUtil.log("Center (Y) " +(float)boundRect[i].center.y*scaleBmpPxToCanvasPx);
                xDist = Math.abs(((float)boundRect[i].center.x*scaleBmpPxToCanvasPx)-320);
                yDist = Math.abs(((float)boundRect[i].center.y*scaleBmpPxToCanvasPx)-240);
                Point vertices[] = new Point[4];;
                boundRect[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    teamUtil.log("Point ("+j+") "+ "x: " + vertices[j].x+ " y: " + vertices[j].y+" RECT ANGLE " + (float)boundRect[i].angle);
                }

                //AREA TESTING
                if(Math.hypot(xDist,yDist)<60){

                    canvas.drawCircle((float)boundRect[i].center.x*scaleBmpPxToCanvasPx, (float)boundRect[i].center.y*scaleBmpPxToCanvasPx, 3,rectPaint);
                    smallestDistFromCenter= Math.hypot(xDist,yDist);
                }
                //teamUtil.log("Smallest Dist From Center: " + smallestDistFromCenter);
            }
        }
        /*
                        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
                Imgproc.circle(matImgDst,boundRect[i].center, 5,PASTEL_GREEN); // Draw a point at the center of the brick
                Point endPoint = new Point();
                endPoint.x = (int) (boundRect[i].center.x + 20 * Math.cos(boundRect[i].angle * 3.14 / 180.0));
                endPoint.y =  (int) (boundRect[i].center.y + 20 * Math.sin(boundRect[i].angle * 3.14 / 180.0));
                Imgproc.line(matImgDst,boundRect[i].center, endPoint, PASTEL_GREEN,3);
                Imgproc.putText(matImgDst, String.valueOf((int)boundRect[i].angle),boundRect[i].center,0,1,PASTEL_GREEN);
                Point vertices[] = new Point[4];;
                boundRect[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    Imgproc.line(matImgDst,vertices[j], vertices[(j+1)%4], PASTEL_RED,3);
                }
                Rect brect = boundRect[i].boundingRect();
                Imgproc.rectangle(matImgDst, brect, PASTEL_PURPLE,2);
*/
    }
}