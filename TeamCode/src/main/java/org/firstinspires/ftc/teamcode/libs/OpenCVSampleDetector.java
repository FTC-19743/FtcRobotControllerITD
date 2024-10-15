package org.firstinspires.ftc.teamcode.libs;

import
        android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
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
import java.util.concurrent.atomic.AtomicInteger;

@Config // Makes Static data members available in Dashboard
public class OpenCVSampleDetector extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Scalar BLACK = new Scalar(0, 0, 0);

    public final int WIDTH = 640;
    public final int HEIGHT = 480;
    static public int TARGET_X = 320;
    static public int TARGET_Y = 160;
    static public int AREA_THRESHOLD = 4000;

    private final double CMS_PER_PIXEL_X = 0; //Set (Will most likely not be linear)
    private final double CMS_PER_PIXEL_Y = 0; //Set (Will most likely not be linear)


    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat erodedMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchyMat = new Mat();

    static public int blurFactor = 10;

    static public int yellowLowH = 15, yellowLowS = 75, yellowLowV = 100;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 20;
    static public int blueLowH = 100, blueLowS = 150, blueLowV = 150;
    static public int blueHighH = 120, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 15;
    static public int redLowH = 0, redLowS = 150, redLowV = 225;
    static public int redHighH = 185, redHighS = 255, redHighV = 255;
    static public int redErosionFactor = 15;

    static public int GAIN = 15;
    static public int EXPOSURE = 15;




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

    public AtomicInteger rectAngle = new AtomicInteger(-1);
    public AtomicInteger rectCenterXOffset = new AtomicInteger(0);
    public AtomicInteger rectCenterYOffset = new AtomicInteger(0);




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
        boolean details = true;
        if (details) teamUtil.log("Sample Detector: Process Frame");

        // Set up the various objects needed to do the image processing
        // This is done here instead of at the class level so these can be adjusted using Acme Dashboard
        Size blurFactorSize = new Size(blurFactor, blurFactor);
        Scalar yellowLowHSV = new Scalar(yellowLowH, yellowLowS, yellowLowV); // lower bound HSV for yellow
        Scalar yellowHighHSV = new Scalar(yellowHighH, yellowHighS, yellowHighV); // higher bound HSV for yellow
        Mat yellowErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * yellowErosionFactor + 1, 2 * yellowErosionFactor + 1),
                new Point(yellowErosionFactor, yellowErosionFactor));
        Scalar blueLowHSV = new Scalar(blueLowH, blueLowS, blueLowV); // lower bound HSV for yellow
        Scalar blueHighHSV = new Scalar(blueHighH, blueHighS, blueHighV); // higher bound HSV for yellow
        Mat blueErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * blueErosionFactor + 1, 2 * blueErosionFactor + 1),
                new Point(blueErosionFactor, blueErosionFactor));
        Scalar redLowHSV = new Scalar(redLowH, redLowS, redLowV); // lower bound HSV for yellow
        Scalar redHighHSV = new Scalar(redHighH, redHighS, redHighV); // higher bound HSV for yellow
        Mat redErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redErosionFactor + 1, 2 * redErosionFactor + 1),
                new Point(redErosionFactor, redErosionFactor));

        Rect cropRect = new Rect(0, 0, frame.width(), frame.height());

        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV

        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise


        switch (targetColor) {
            case YELLOW:
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, yellowErosionElement);
                break;
            case RED:
                Core.inRange(blurredMat, redLowHSV, redHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);
                break;
            case BLUE:
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, blueErosionElement);
                break;
        }
        Imgproc.rectangle(erodedMat, cropRect, BLACK, 2); // black frame to help with erosion and boundaries.

        Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges


        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around white areas

        if (contours.isEmpty()) {
            foundOne.set(false);
            return null;
        }

        // We found at least one blob of the right color
        foundOne.set(true); // TODO: Have we actually found one yet?  Might be too small?
        MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
        RotatedRect[] rotatedRect = new RotatedRect[contours.size()];
        double closestAreaSelection = 1000; // TODO: What is this?
        int closestAreaSelectionNum = -1;
        double xDistCenter;
        double yDistCenter;

        //Create Rotated Rectangles from contours and identify the one closest to the target that meets the threshold size
        for (int i = 0; i < contours.size(); i++) {
            contoursPoly[i] = new MatOfPoint2f();
            Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
            rotatedRect[i] = Imgproc.minAreaRect(contoursPoly[i]);

            // Is the closest one to the target we have seen so far?
            xDistCenter = Math.abs(rotatedRect[i].center.x - TARGET_X);
            yDistCenter = Math.abs(rotatedRect[i].center.y - TARGET_Y); // TODO, this was 140 instead of the correct TARGET_Y...
            if (Math.hypot(xDistCenter, yDistCenter) < closestAreaSelection && rotatedRect[i].size.area() > AREA_THRESHOLD) {
                closestAreaSelection = Math.hypot(xDistCenter, yDistCenter);
                closestAreaSelectionNum = i;
            }
        }

        if (closestAreaSelectionNum == -1) { // nothing big enough
            if (details) teamUtil.log("Saw blobs but nothing big enough");
            foundOne.set(false);
            return null;
        }

        //Find the point with the lowest y coordinate
        Point vertices1[] = new Point[4];
        rotatedRect[closestAreaSelectionNum].points(vertices1); // get the 4 corners of the rotated rectangle
        int lowestPixel = 0;
        for (int j = 1; j < 4; j++) {
            if (vertices1[j].y > vertices1[lowestPixel].y) {
                lowestPixel = j;
            }
        }

        //Find the point closest to lowest point
        int closestPixel = 0;
        double closestDist = 1000;
        for (int k = 0; k < 4; k++) {
            if (k == lowestPixel) {
            } else {
                double xDiff = Math.abs(vertices1[lowestPixel].x - vertices1[k].x);
                double yDiff = Math.abs(vertices1[lowestPixel].x - vertices1[k].y);
                if (Math.hypot(xDiff, yDiff) < closestDist) {
                    closestDist = Math.hypot(xDiff, yDiff);
                    closestPixel = k;
                }
            }

        }
        double xDist = Math.abs(vertices1[closestPixel].x - vertices1[lowestPixel].x);
        double yDist = Math.abs(vertices1[closestPixel].y - vertices1[lowestPixel].y);
        double realAngle = Math.toDegrees(Math.atan(yDist / xDist));
        if (vertices1[closestPixel].x < vertices1[lowestPixel].x) {
            realAngle += 90;
        } else {
            realAngle = 90 - realAngle;
        }
        /*
        //TODO POssible change later
        if (Math.abs((int) realAngle - rectAngle.get()) > 2) {
            rectAngle.set((int) realAngle);
        }

        if (Math.abs((int) rotatedRect[closestAreaSelectionNum].center.y - rectCenterYOffset.get()) > 2) {
            rectCenterYOffset.set(-1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y));
        }
        if (Math.abs((int) rotatedRect[closestAreaSelectionNum].center.x - rectCenterXOffset.get()) > 2) {
            rectCenterXOffset.set((int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X);
        }


         */
        rectCenterYOffset.set(-1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y));
        rectCenterXOffset.set((int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X);

        rectAngle.set((int) realAngle);
        if (details) teamUtil.log("Real Angle" + realAngle);
        if (details) teamUtil.log("Real Angle" + realAngle + "Lowest: " + vertices1[lowestPixel].x + "," + vertices1[lowestPixel].y+"Closest: " + vertices1[closestPixel].x+ "," +vertices1[closestPixel].y);

        return rotatedRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        // Use the appropriate background if we are viewing the pipeline
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
            RotatedRect[] rotatedRect = (RotatedRect[]) userContext;

            Paint rectPaint = new Paint();
            rectPaint.setColor(Color.MAGENTA);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 6);
            Paint anglePaint = new Paint();
            anglePaint.setColor(Color.CYAN);
            anglePaint.setStyle(Paint.Style.STROKE);
            anglePaint.setTextSize(20);
            anglePaint.setStrokeWidth(scaleCanvasDensity * 6);
            Paint centerPaint = new Paint();
            centerPaint.setColor(Color.BLACK);
            centerPaint.setStyle(Paint.Style.STROKE);
            centerPaint.setStrokeWidth(scaleCanvasDensity * 6);
            canvas.drawCircle((float)TARGET_X*scaleBmpPxToCanvasPx, (float)TARGET_Y*scaleBmpPxToCanvasPx, 10,rectPaint);


            for (int i = 0; i < rotatedRect.length; i++) {

                // Draw Center
                canvas.drawCircle((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx, (float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx, 5,centerPaint);

                // Draw rotated Rectangle
                Point vertices[] = new Point[4];
                rotatedRect[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    canvas.drawLine((float)vertices[j].x*scaleBmpPxToCanvasPx,(float)vertices[j].y*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].x*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].y*scaleBmpPxToCanvasPx,rectPaint);
                }

                // Draw angle vector
                int endX = (int) (rotatedRect[i].center.x + 20 * Math.cos(rectAngle.get() * 3.14 / 180.0));
                int endY =  (int) (rotatedRect[i].center.y + 20 * Math.sin(rectAngle.get() * 3.14 / 180.0));
                canvas.drawLine((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,(float)endX*scaleBmpPxToCanvasPx,(float)endY*scaleBmpPxToCanvasPx,rectPaint);
                //Imgproc.putText(matImgDst, String.valueOf((int)boundRect[i].angle),boundRect[i].center,0,1,PASTEL_GREEN);
                canvas.drawText(Integer.toString(rectAngle.get()),(float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,anglePaint);

            }
        }
    }



    public void OLDonDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        boolean details = false;
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
            rectPaint.setColor(Color.MAGENTA);
            rectPaint.setStyle(Paint.Style.STROKE);
            rectPaint.setStrokeWidth(scaleCanvasDensity * 8);
            Paint centerPaint = new Paint();
            centerPaint.setColor(Color.BLACK);
            centerPaint.setStyle(Paint.Style.STROKE);
            centerPaint.setStrokeWidth(scaleCanvasDensity * 8);
            double smallestDistFromCenter=0;
            double xDist;
            double yDist;
            for (int i = 0; i < boundRect.length; i++) {
                //Old Normal Circle Drawing
                //canvas.drawCircle((float)boundRect[i].center.x*scaleBmpPxToCanvasPx, (float)boundRect[i].center.y*scaleBmpPxToCanvasPx, 3,rectPaint);
                if(details) { // TODO: Why would this be here and not in process frame?
                    teamUtil.log("Center (X) " + (float) boundRect[i].center.x * scaleBmpPxToCanvasPx);
                    teamUtil.log("Height" + (float) boundRect[i].size.height);
                    teamUtil.log("Height" + (float) boundRect[i].size.width);
                    teamUtil.log("Center (Y) " + (float) boundRect[i].center.y * scaleBmpPxToCanvasPx);
                }
                xDist = Math.abs(((float) boundRect[i].center.x * scaleBmpPxToCanvasPx) - TARGET_X);
                yDist = Math.abs(((float) boundRect[i].center.y * scaleBmpPxToCanvasPx) - TARGET_Y);
                Point vertices[] = new Point[4];
                ;
                boundRect[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    double lowestY = 0;
                    double lowestX = 0;
                    int lowestPixel = 0;

                    for (int k = 0; k < 4; k++) {
                        if (vertices[k].y > lowestY) {
                            lowestY = vertices[k].y;
                            lowestX = vertices[k].x;
                            lowestPixel = k;

                        }
                        //System.out.println("Point ("+k+") "+ "x: " + vertices1[k].x+ " y: " + vertices1[k].y+" RECT ANGLE " + (float)boundRect[i].angle);
                    }
                    double closestDist = 1000;
                    double closestY = 1000;
                    double closestX = 0;
                    int closestPixel = 0;



                    for (int k = 0; k < 4; k++) {
                        if (vertices[k].y == lowestY) {
                        } else {
                        	/*
                            if(Math.abs(lowestY - vertices1[k].y)<closestY){
                                closestY=Math.abs(lowestY - vertices1[k].y);
                                closestX= vertices1[k].x;
                                closestPixel=k;

                            }
                            */
                            double xDiff = Math.abs(lowestX - vertices[k].x);
                            double yDiff = Math.abs(lowestY - vertices[k].y);

                            if (Math.hypot(xDiff, yDiff) < closestDist) {
                                closestDist = Math.hypot(xDiff, yDiff);
                                closestPixel = k;
                            }
                        }
                    }
                    canvas.drawLine((float)vertices[closestPixel].x,(float)vertices[closestPixel].y,(float)vertices[closestPixel].x,(float)vertices[lowestPixel].y,rectPaint);
                    canvas.drawLine((float)vertices[lowestPixel].x,(float)vertices[lowestPixel].y,(float)vertices[closestPixel].x,(float)vertices[lowestPixel].y,rectPaint);



                    //System.out.println("Lowest Y: " + lowestY + "Lowest X: " + lowestX);
                    /*
                    Imgproc.line(matImgDst,vertices1[j], vertices1[(j+1)%4], PASTEL_RED,3);
                    Imgproc.circle(matImgDst,vertices1[lowestPixel],3,PASTEL_GREEN,14);
                    Imgproc.circle(matImgDst,vertices1[closestPixel],3,PASTEL_PURPLE,14);

                     */

                    double realAngle = Math.toDegrees(Math.atan(yDist / xDist));
                    if (vertices[closestPixel].x < vertices[lowestPixel].x) {
                        realAngle += 90;
                    } else {
                        realAngle = 90 - realAngle;
                    }


                    //System.out.println("Points"+vertices1[j]);
                    if(details)teamUtil.log("Real Angle" + realAngle);
                }
                teamUtil.log("Area: " + (float)boundRect[i].size.area());
                //canvas.drawCircle((float)lowestX, (float)lowestY, 3,rectPaint);
                //canvas.drawCircle((float)closestX, (float)closestY, 3,rectPaint);

                canvas.drawCircle((float)boundRect[i].center.x*scaleBmpPxToCanvasPx, (float)boundRect[i].center.y*scaleBmpPxToCanvasPx, 3,rectPaint);
                for (int j = 0; j < 4; j++) {
                    canvas.drawLine((float)vertices[j].x*scaleBmpPxToCanvasPx,(float)vertices[j].y*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].x*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].y*scaleBmpPxToCanvasPx,rectPaint);
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