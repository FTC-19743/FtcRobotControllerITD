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
import org.opencv.core.CvType;
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
    Scalar GREEN = new Scalar(0, 255, 0);


    public final int WIDTH = 640;
    public final int HEIGHT = 480;
    Rect obscureRect = new Rect(0,300,WIDTH,HEIGHT-300);
    Rect cropRect = new Rect(0, 0, WIDTH, HEIGHT);

    static public int TARGET_X = 334;
    static public int TARGET_Y = 160;
    static public int AREA_THRESHOLD = 2000;

    private final double CMS_PER_PIXEL_X = 0; //Set (Will most likely not be linear)
    private final double CMS_PER_PIXEL_Y = 0; //Set (Will most likely not be linear)

/* No light Arducam
    static public int GAIN = 127;
    static public int EXPOSURE = 1;
    static public int TEMPERATURE = 4600;
    static public boolean APEXPOSURE = false;
    static public boolean AEPRIORITY = false;
    static public boolean WHITEBALANCEAUTO = true;
    static public int blurFactor = 10;

    static public int yellowLowH = 10, yellowLowS = 60, yellowLowV = 100;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 20;
    static public int blueLowH = 100, blueLowS = 50, blueLowV = 20;
    static public int blueHighH = 140, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 20;
    static public int redLowHA = -1, redLowSA = 100, redLowVA = 100;
    static public int redHighHA = 8, redHighSA = 255, redHighVA = 255;
    static public int redLowHB = 140, redLowSB = 100, redLowVB = 100;
    static public int redHighHB = 180, redHighSB = 255, redHighVB = 255;
    static public int redErosionFactor = 20;
*/
    static public boolean AFOCUS = true;
    static public boolean APEXPOSURE = false;
    static public boolean AEPRIORITY = false;
    static public boolean WHITEBALANCEAUTO = true;
    static public int FOCUSLENGTH =125;
    static public int GAIN = 150;
    static public int EXPOSURE = 1;
    static public int TEMPERATURE = 2800;

    static public int blurFactor = 10;

    static public int yellowLowH = 10, yellowLowS = 100, yellowLowV = 150;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 20;
    static public int blueLowH = 90, blueLowS = 120, blueLowV = 30; // low was 10
    static public int blueHighH = 130, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 20;
    static public int rbyLowH = -1, rbyLowS = 150, rbyLowV = 130;
    static public int rbyHighH = 180, rbyHighS = 255, rbyHighV = 255;
    static public int redErosionFactor = 20;
    static public int redDilutionFactor = 10;

    //static public int CLOSEFACTOR = 20;

    private int sampleX = TARGET_X;
    private int sampleY = TARGET_Y;
    static public int SAMPLE_SIZE = 1;
    //public double[] samplePixel = new double[3];
    Rect sampleRect;
    public Scalar sampleAvgs = new Scalar(0, 0, 0); // Average HSV values in sample rectangle
    int targetIndex = 0;



    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat thresholdMatAll = new Mat();
    private Mat thresholdMatYellow = new Mat();
    private Mat thresholdMatBlue = new Mat();
    private Mat thresholdMatYB = new Mat();
    private Mat invertedMat = new Mat();
    private Mat floodFillMask = new Mat();
    private Mat erodedMat = new Mat();
    private Mat edgesMat = new Mat();
    private Mat hierarchyMat = new Mat();


    public enum TargetColor {
        YELLOW,
        RED,
        BLUE
    }
    static public TargetColor targetColor = TargetColor.YELLOW;
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
        INVERTED,
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

    public void sampleUp(int step) {
        sampleY = sampleY - step;
        if (sampleY < 0) sampleY = 0;
    }
    public void sampleLeft(int step) {
        sampleX = sampleX- step;
        if (sampleX< 0) sampleX = 0;
    }
    public void sampleDown(int step) {
        sampleY = sampleY + step;
        if (sampleY > HEIGHT-SAMPLE_SIZE-1) sampleY = HEIGHT-SAMPLE_SIZE-1;
    }
    public void sampleRight(int step) {
        sampleX = sampleX + step;
        if (sampleX> WIDTH-SAMPLE_SIZE-1) sampleX= WIDTH-SAMPLE_SIZE-1;
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
        Scalar rbyLowHSV = new Scalar(rbyLowH, rbyLowS, rbyLowV); // lower bound HSV for yellow
        Scalar rbyHighHSV = new Scalar(rbyHighH, rbyHighS, rbyHighV); // higher bound HSV for yellow
        Mat redErosionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redErosionFactor + 1, 2 * redErosionFactor + 1),
                new Point(redErosionFactor, redErosionFactor));

        Mat redDilutionElement = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(2 * redDilutionFactor + 1, 2 * redDilutionFactor + 1),
                new Point(redDilutionFactor, redDilutionFactor));

        Imgproc.rectangle(frame, obscureRect, BLACK, -1); // Cover view of robot

        Imgproc.cvtColor(frame, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV

        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise


        switch (targetColor) {
            case YELLOW:
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMat);
                //Imgproc.morphologyEx(thresholdMat, closedMat,Imgproc.MORPH_CLOSE, closeElement);
                Imgproc.erode(thresholdMat, erodedMat, yellowErosionElement);
                break;
            case BLUE:
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMat);
                //Imgproc.morphologyEx(thresholdMat, closedMat,Imgproc.MORPH_CLOSE, closeElement);
                Imgproc.erode(thresholdMat, erodedMat, blueErosionElement);
                break;
            case RED:
                Core.inRange(blurredMat, rbyLowHSV, rbyHighHSV, thresholdMatAll); // Get Red and Yellow
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMatYellow);
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMatBlue);
                Core.add(thresholdMatBlue, thresholdMatYellow, thresholdMatYB); // combine yellow and blue threshold mat
                Core.subtract(thresholdMatAll, thresholdMatYB, thresholdMat); // Then subtract from all to get red

                Core.bitwise_not(thresholdMat,invertedMat);
                Imgproc.rectangle(invertedMat, cropRect, new Scalar(255), 2); // white frame to provide seed and propagation for fill.
                floodFillMask = new Mat();
                Imgproc.floodFill(invertedMat, floodFillMask, new Point(0,0),BLACK);
                //Imgproc.Canny(dilutedMat, edgesMat, 100, 300); // find edges
                List<MatOfPoint> contours = new ArrayList<>();
                contours.clear(); // empty the list from last time
                Imgproc.findContours(invertedMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around holes we need to fill
                //for (int i = 0; i < contours.size(); i++) {
                Imgproc.drawContours(thresholdMat,contours,-1,BLACK,-1); // fill the holes back on the original threshold mat

                //Core.add(dilutedMat,thresholdMat, closedMat);
                //Imgproc.dilate(thresholdMat, dilutedMat, redDilutionElement);
                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);
                break;

        }
        Imgproc.rectangle(erodedMat, cropRect, BLACK, 2); // black frame to help with edges and boundaries.

        Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges


        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear(); // empty the list from last time
        Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around white areas

        if (contours.isEmpty()) {
            foundOne.set(false);
            return null;
        }

        // We found at least one blob of the right color
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
        foundOne.set(true);
        rectCenterYOffset.set(-1 * ((int) rotatedRect[closestAreaSelectionNum].center.y - TARGET_Y));
        rectCenterXOffset.set((int) rotatedRect[closestAreaSelectionNum].center.x - TARGET_X);
        rectAngle.set((int) realAngle);
        targetIndex = closestAreaSelectionNum;

        if (details) teamUtil.log("Real Angle" + realAngle + "Lowest: " + vertices1[lowestPixel].x + "," + vertices1[lowestPixel].y+"Closest: " + vertices1[closestPixel].x+ "," +vertices1[closestPixel].y);

        return rotatedRect;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {

        //samplePixel = blurredMat.get(sampleX,sampleY);
        sampleRect = new Rect(sampleX-SAMPLE_SIZE/2,sampleY-SAMPLE_SIZE/2,SAMPLE_SIZE,SAMPLE_SIZE);
        sampleAvgs = getAverages(blurredMat,sampleRect);

        // Use the appropriate background if we are viewing the pipeline
        if (viewingPipeline) {
            Bitmap bmp = Bitmap.createBitmap(HSVMat.cols(), HSVMat.rows(), Bitmap.Config.ARGB_8888);
            switch (stageToRenderToViewport) {
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                case INVERTED: {
                    if (targetColor == TargetColor.RED) {
                        Utils.matToBitmap(invertedMat, bmp);
                    } else {
                        Utils.matToBitmap(thresholdMat, bmp);
                    }
                    break;
                }
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case ERODED: { Utils.matToBitmap(erodedMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edgesMat, bmp); break;}
                default: {}
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(WIDTH*scaleBmpPxToCanvasPx), (int)(HEIGHT*scaleBmpPxToCanvasPx), false);
            canvas.drawBitmap(resizedBitmap, 0,0,null);
        }



        Paint samplePaint = new Paint();
        samplePaint.setColor(Color.GREEN);
        samplePaint.setStyle(Paint.Style.STROKE);
        samplePaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawCircle((float)TARGET_X*scaleBmpPxToCanvasPx, (float)TARGET_Y*scaleBmpPxToCanvasPx, 10,samplePaint);
        canvas.drawRect((float)sampleRect.tl().x*scaleBmpPxToCanvasPx, (float)sampleRect.tl().y*scaleBmpPxToCanvasPx, (float)sampleRect.br().x*scaleBmpPxToCanvasPx, (float)sampleRect.br().y*scaleBmpPxToCanvasPx,samplePaint);

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

            for (int i = 0; i < rotatedRect.length; i++) {

                // Draw rotated Rectangle
                Point vertices[] = new Point[4];
                rotatedRect[i].points(vertices);
                for (int j = 0; j < 4; j++) {
                    canvas.drawLine((float)vertices[j].x*scaleBmpPxToCanvasPx,(float)vertices[j].y*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].x*scaleBmpPxToCanvasPx,(float)vertices[(j+1)%4].y*scaleBmpPxToCanvasPx,rectPaint);
                }
                if (targetIndex == i) {
                    // Draw Center
                    canvas.drawCircle((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx, (float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx, 5,centerPaint);

                    // Draw angle vector
                    int endX = (int) (rotatedRect[i].center.x + 20 * Math.cos(rectAngle.get() * 3.14 / 180.0));
                    int endY =  (int) (rotatedRect[i].center.y + 20 * Math.sin(rectAngle.get() * 3.14 / 180.0));
                    canvas.drawLine((float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,(float)endX*scaleBmpPxToCanvasPx,(float)endY*scaleBmpPxToCanvasPx,rectPaint);
                    //Imgproc.putText(matImgDst, String.valueOf((int)boundRect[i].angle),boundRect[i].center,0,1,PASTEL_GREEN);
                    canvas.drawText(Integer.toString(rectAngle.get()),(float)rotatedRect[i].center.x*scaleBmpPxToCanvasPx,(float)rotatedRect[i].center.y*scaleBmpPxToCanvasPx,anglePaint);
                }

            }
        }
    }
}