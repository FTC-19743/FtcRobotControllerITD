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
import java.util.concurrent.atomic.AtomicInteger;

public class OpenCVSampleDetector extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;

    Scalar BLACK = new Scalar(0, 0, 0);
    Scalar PASTEL_GREEN  = new Scalar(204, 255, 204);
    Scalar PASTEL_RED = new Scalar(204, 204, 255);
    Scalar PASTEL_PURPLE = new Scalar(255, 204, 204);

    public final int WIDTH = 640;
    public final int HEIGHT = 480;

    private final double CMS_PER_PIXEL_X = 0; //Set (Will most likely not be linear)
    private final double CMS_PER_PIXEL_Y = 0; //Set (Will most likely not be linear)


    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat erodedMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchyMat = new Mat();

    private int blurFactor = 10;
    private Size blurFactorSize = new Size(blurFactor,blurFactor);

    private int yellowLowH = 15, yellowLowS = 75, yellowLowV = 100;
    private int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    private int yellowErosionFactor = 20;
    Scalar yellowLowHSV = new Scalar(yellowLowH, yellowLowS, yellowLowV); // lower bound HSV for yellow
    Scalar yellowHighHSV = new Scalar(yellowHighH, yellowHighS, yellowHighV); // higher bound HSV for yellow
    Mat yellowErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * yellowErosionFactor + 1, 2 * yellowErosionFactor + 1),
            new Point(yellowErosionFactor, yellowErosionFactor));
    // TODO Repeat above section for Red
    public int blueLowH = 100, blueLowS = 150, blueLowV = 150;
    public int blueHighH = 120, blueHighS = 255, blueHighV = 255;
    public int blueErosionFactor = 15;
    public Scalar blueLowHSV = new Scalar(blueLowH, blueLowS, blueLowV); // lower bound HSV for yellow
    public Scalar blueHighHSV = new Scalar(blueHighH, blueHighS, blueHighV); // higher bound HSV for yellow
    Mat blueErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * blueErosionFactor + 1, 2 * blueErosionFactor + 1),
            new Point(blueErosionFactor, blueErosionFactor));

    public int redLowH = 0, redLowS = 150, redLowV = 225;
    public int redHighH = 185, redHighS = 255, redHighV = 255;
    public int redErosionFactor = 15;
    public Scalar redLowHSV = new Scalar(redLowH, redLowS, redLowV); // lower bound HSV for yellow
    public Scalar redHighHSV = new Scalar(redHighH, redHighS, redHighV); // higher bound HSV for yellow
    Mat redErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redErosionFactor + 1, 2 * redErosionFactor + 1),
            new Point(redErosionFactor, redErosionFactor));


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
                Core.inRange(blurredMat, redLowHSV, redHighHSV, thresholdMat);
                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);
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
            foundOne.set(true);
            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            RotatedRect[] boundRect = new RotatedRect[contours.size()];
            double largestAreaSelection = 0;
            int largestAreaSelectionNum = 0;
            //Size Check; largest area returned
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
                if(boundRect[i].size.area()>largestAreaSelection){
                    largestAreaSelection=boundRect[i].size.area();
                    largestAreaSelectionNum = i;
                }

            }
            /////////////AREA SELECTION THRESHOLD SELECTED AT CAMERA HEIGHT OF ABOUT 7.5 INCHES (IS 4000 AT THIS HEIGHT)
            /////////////MAKE SURE TO CHANGE IF CAMERA HEIGHT CHANGED
            if(largestAreaSelection<4000){
                teamUtil.log("Too Small");
                rectAngle.set(-10);
                if (details) teamUtil.log("No Angle");
                rectCenterYOffset.set(-1000);
                rectCenterXOffset.set(-1000);

            }
            else{
                //Also finds closest Area to Center of camera
                double closestAreaSelection = 1000;
                int closestAreaSelectionNum = 0;
                double xDistCenter;
                double yDistCenter;
                for (int i = 0; i < contours.size(); i++) {
                    contoursPoly[i] = new MatOfPoint2f();
                    Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                    boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
                    xDistCenter = Math.abs(boundRect[i].center.x-320);
                    yDistCenter = Math.abs(boundRect[i].center.y-240);
                    if(Math.hypot(xDistCenter,yDistCenter)<closestAreaSelection) {
                        closestAreaSelection = Math.hypot(xDistCenter, yDistCenter);
                        closestAreaSelectionNum = i;
                    }
                }
                //Decides which rectangle to use
                //If the closest Area is not the Largest area then chooses to use closest area
                //Makes sure that closest area is not a tiny piece of noise and then moves on
                int selectedRect = 0;
                if(closestAreaSelectionNum!=largestAreaSelectionNum){
                    /////////////AREA SELECTION THRESHOLD SELECTED AT CAMERA HEIGHT OF ABOUT 7.5 INCHES (IS 4000 AT THIS HEIGHT)
                    /////////////MAKE SURE TO CHANGE IF CAMERA HEIGHT CHANGED
                    if(closestAreaSelection<4000){
                        selectedRect=largestAreaSelectionNum;
                    }
                    else{
                        selectedRect=closestAreaSelectionNum;
                    }
                }else{
                    selectedRect=largestAreaSelectionNum;
                }
                //Below for loop is used for all
                for (int i = 0; i < contours.size(); i++) {
                    if(i!=selectedRect){
                    }
                    else{
                        contoursPoly[i] = new MatOfPoint2f();
                        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                        boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);

                        Point vertices1[] = new Point[4];;

                        boundRect[i].points(vertices1);

                        for (int j = 0; j < 4; j++) {
                            double lowestY = 0;
                            double lowestX = 0;
                            int lowestPixel = 0;

                            for (int k = 0; k < 4; k++) {
                                if(vertices1[k].y>lowestY){
                                    lowestY=vertices1[k].y;
                                    lowestX=vertices1[k].x;
                                    lowestPixel=k;

                                }
                            }
                            double closestDist = 1000;
                            double closestY =1000;
                            double closestX=0;
                            int closestPixel = 0;


                            for (int k = 0; k < 4; k++) {
                                if(vertices1[k].y==lowestY){
                                }else{

                                    double xDiff = Math.abs(lowestX - vertices1[k].x);
                                    double yDiff = Math.abs(lowestY - vertices1[k].y);

                                    if(Math.hypot(xDiff,yDiff)<closestDist){
                                        closestDist = Math.hypot(xDiff,yDiff);
                                        closestPixel = k;
                                    }
                                }
                            }

                            double xDist = Math.abs(vertices1[closestPixel].x-vertices1[lowestPixel].x);
                            double yDist = Math.abs(vertices1[closestPixel].y-vertices1[lowestPixel].y);
                            double realAngle = Math.toDegrees(Math.atan(yDist/xDist));
                            if(vertices1[closestPixel].x<vertices1[lowestPixel].x) {
                                realAngle+=90;
                            }
                            else {
                                realAngle = 90-realAngle;
                            }
                            if(Math.abs(xDist)<1){
                                realAngle=90;
                            }

                            /*
                            if ((Math.abs((int)realAngle-rectAngle.get())>80)&&rectAngle.get()>0) {
                                realAngle = 90;
                            }

                             */
                            if(Math.abs((int)realAngle-rectAngle.get())>2){
                                rectAngle.set((int)realAngle);
                            }

                            if(Math.abs((int)boundRect[i].center.y-rectCenterYOffset.get())>2){
                                rectCenterYOffset.set(-1*((int)boundRect[i].center.y-320));
                            }
                            if(Math.abs((int)boundRect[i].center.x-rectCenterXOffset.get())>2){
                                rectCenterXOffset.set((int)boundRect[i].center.x-320);
                            }
                            //System.out.println("Points"+vertices1[j]);
                            teamUtil.log("Real Angle"+realAngle);
                            //Imgproc.putText(matImgDst, String.valueOf(realAngle),vertices1[lowestPixel],0,1,PASTEL_GREEN
                        }
                    }

                }
            }

            return boundRect; // return array of rotated rectangles we found
        }  else {
            foundOne.set(false);

            if (details) teamUtil.log("No Detections");
            rectAngle.set(-10);
            if (details) teamUtil.log("No Angle");
            rectCenterYOffset.set(-1000);
            rectCenterXOffset.set(-1000);
            if (details) teamUtil.log("No Center");



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
                    teamUtil.log("Real Angle" + realAngle);
                }
                //canvas.drawCircle((float)lowestX, (float)lowestY, 3,rectPaint);
                //canvas.drawCircle((float)closestX, (float)closestY, 3,rectPaint);


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