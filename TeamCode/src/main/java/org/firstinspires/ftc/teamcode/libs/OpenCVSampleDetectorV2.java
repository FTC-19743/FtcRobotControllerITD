package org.firstinspires.ftc.teamcode.libs;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
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
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicLong;


@Config // Makes Static data members available in Dashboard
public class OpenCVSampleDetectorV2 extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;



    Scalar BLACK = new Scalar(0, 0, 0);
    Scalar GREEN = new Scalar(0, 255, 0);


    static public final int WIDTH = 640;
    static public final int HEIGHT = 480;
    static public int OBSCURE_HEIGHT = 460;
    Rect cropRect = new Rect(0, 0, WIDTH, HEIGHT);

    static public int TARGET_X = 336;
    static public int TARGET_Y = 190;
    static public double CAMERA_OFFSET_Y = TARGET_Y-HEIGHT/2f;
    static public double CAMERA_OFFSET_X = TARGET_X-WIDTH/2f;
    static public int MIN_AREA_THRESHOLD = 2000;
    static public int MAX_AREA_THRESHOLD = 15000; // was 13000 for meet #2 build
    static public int GOLDILOCKS_ZONE_RADIUS = 160;

    public static float CAM_ANGLE_ADJUST_NEG_Y = .82f;
    public static float CAM_ANGLE_ADJUST_POS_Y = .84f;
    public static float CAM_ANGLE_ADJUST_POS_Y_B = -10f;
    public static float CAM_ANGLE_ADJUST_MID_Y = .85f;

    public static float CAM_ANGLE_ADJUST_NEG_X = .82f;
    public static float CAM_ANGLE_ADJUST_NEG_X_B = 0f;
    public static float CAM_ANGLE_ADJUST_POS_X = .84f;
    public static float CAM_ANGLE_ADJUST_MID_X = .85f;

    double[][] cameraMatrixData = {
            { 478.3939243528238, 0.0, 333.6082048642555 },
            { 0.0, 470.9276030162481, 297.451893471244 },
            { 0.0, 0.0, 1.0 }
    };

    // Define your distortion coefficients constants
    double[] distCoeffsData = { -0.3219047808733794, 0.09609610660662812, -0.04785830392770931, -0.0050045386058039, -0.05072346571900022 };


    Mat cameraMatrix = new Mat(3, 3, CvType.CV_64F);
    Mat distCoeffs = new Mat(1, 5, CvType.CV_64F);

    public List<Point[]> modPolys;

    public static int polyEp = 10;
    public static double AREA_THRESHOLD = 10000;
    public static double EXTERNAL_THRESHOLD = 10;
    public static double SEGMENT_THRESHOLD = 15;
    public static double STRAIGHT_THRESHOLD = 10f; // degrees
    public static double RIGHT_ANGLE_TOLERANCE = 12f;// degrees
    public static double LONG_LENGTH_TARGET = 170;
    public static double LONG_LENGTH_THRESHOLD = LONG_LENGTH_TARGET*0.2f;
    public static double SHORT_LENGTH_TARGET = 65;
    public static double SHORT_LENGTH_THRESHOLD = SHORT_LENGTH_TARGET*0.3f;



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
    static public int EXPOSURE = 4; // With Adafruit 12 led ring at full white (no RGB)
    static public int TEMPERATURE = 2800;

    static public int blurFactor = 10;
    static public boolean undistort = true;

    /* 4 leds (Meet 1)
    static public int yellowLowH = 15, yellowLowS = 85, yellowLowV = 150;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 20;
    static public int blueLowH = 90, blueLowS = 100, blueLowV = 30; // low was 10
    static public int blueHighH = 130, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 20;
    static public int rbyLowH = -1, rbyLowS = 130, rbyLowV = 115;
    static public int rbyHighH = 180, rbyHighS = 255, rbyHighV = 255;
    static public int redErosionFactor = 20;
    static public int redDilutionFactor = 10;
     */
    // Adafruit Ring max white (no rgb) at manual exposure 4
    static public int yellowLowH = 10, yellowLowS = 100, yellowLowV = 170;
    static public int yellowHighH = 35, yellowHighS = 255, yellowHighV = 255;
    static public int yellowErosionFactor = 10;
    static public int blueLowH = 90, blueLowS = 80, blueLowV = 90; // lowv was 70 meet #2 build
    static public int blueHighH = 130, blueHighS = 255, blueHighV = 255;
    static public int blueErosionFactor = 10;
    static public int rbyLowH = -1, rbyLowS = 150, rbyLowV = 125;
    static public int rbyHighH = 180, rbyHighS = 255, rbyHighV = 255;
    static public int redErosionFactor = 10;
    static public int redDilutionFactor = 10;


    //static public int CLOSEFACTOR = 20;

    public int sampleX = TARGET_X;
    public int sampleY = TARGET_Y;
    public static int FOUND_ONE_RIGHT_THRESHOLD = 620;
    public static int FOUND_ONE_LEFT_THRESHOLD = 20;

    static public int SAMPLE_SIZE = 1;
    //public double[] samplePixel = new double[3];
    Rect sampleRect;
    public Scalar sampleAvgs = new Scalar(0, 0, 0); // Average HSV values in sample rectangle
    int targetIndex = 0;



    private Mat HSVMat  = new Mat();
    private Mat blurredMat = new Mat();
    private Mat thresholdMat = new Mat();
    private Mat thresholdMatAll = new Mat();
    private Mat undistortedMat = new Mat();
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
    public AtomicBoolean processedFrame = new AtomicBoolean(false);
    public AtomicLong processTime = new AtomicLong(0);
    private long startProcessingTime = 0;

    public AtomicBoolean outsideUseableCameraRange = new AtomicBoolean(false);
    // Data about the located Sample
    //old atomic integers
    public AtomicInteger rectAngle = new AtomicInteger(-1);
    public AtomicInteger rectCenterXOffset = new AtomicInteger(0);
    public AtomicInteger rectCenterYOffset = new AtomicInteger(0);
    public AtomicInteger rectArea = new AtomicInteger(0);



    public static class FrameData {
        public int rectAngle =-1;
        public int rectCenterXOffset = 0;
        public int rectCenterYOffset = 0;
        public int adjRectCenterXOffset = 0;
        public int adjRectCenterYOffset = 0;
        public int rectArea = 0;
        public RotatedRect rrect;
        public int lowestPixelX = 0;
        public int lowestPixelY = 0;

        public int closestPixelX = 0;
        public int closestPixelY = 0;

        public long captureTime = 0;
        public long processingTime = 0;
        public long processingTime2 = 0;

    }
    public ConcurrentLinkedQueue<FrameData> frameDataQueue = new ConcurrentLinkedQueue<FrameData>();


    public boolean viewingPipeline = false;
    enum Stage {
        RAW_IMAGE,
        UNDISTORTED,
        HSV,
        BLURRED,
        THRESHOLD,
        ERODED,
        EDGES,
    }
    private Stage stageToRenderToViewport = Stage.UNDISTORTED;
    private Stage[] stages = Stage.values();

    private VisionPortal myPortal;

    class Context {
        RotatedRect[] rots;
        List<MatOfPoint> contours;
    }

    public OpenCVSampleDetectorV2() {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVSampleDetector processor");
        for (int i=0;i<3;i++)
            cameraMatrix.put(i,0, cameraMatrixData[i]); // preload calibration data
        distCoeffs.put(0, 0, distCoeffsData);

        teamUtil.log("Initializing OpenCVSampleDetector processor - FINISHED");
    }
    public void setVisionPortal(VisionPortal portal) {
        myPortal = portal;

    }

    public String frameString(FrameData frame) {
        if (frame == null) {
            return "No Detection";
        } else {
            return "Detection x,y: " + frame.rectCenterXOffset + ", " + frame.rectCenterYOffset
                    + " Angle: " + frame.rectAngle
                    + " Area: " + frame.rectArea
                    + " Processing Time: " + frame.processingTime
                    + " Adjusted Target: " + frame.adjRectCenterXOffset + "," + frame.adjRectCenterYOffset
                    + " Lowest Pixel: " + frame.lowestPixelX + "," + frame.lowestPixelY
                    + " Closest Pixel: " + frame.closestPixelX + "," + frame.closestPixelY
                    //+ " Capture Time: " + frame.captureTime
                    //+ " Processing Time2: " + frame.processingTime2
                    ;
        }
    }

    public void outputTelemetry () {
        telemetry.addLine("Processed Frame: " + processedFrame.get());
        //telemetry.addLine("queue size: "+ frameDataQueue.size());
        telemetry.addLine(frameString(frameDataQueue.peek()));
        //teamUtil.log(frameString(frameDataQueue.peek()));
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
        frameDataQueue.clear(); // remove any existing frame data from queue
        foundOne.set(false);
        processedFrame.set(false);
    }

    public FrameData processNextFrame(boolean waitForDetection, boolean turnOffProcessor, boolean captureImages, long timeOut){
        boolean details = true;
        teamUtil.log("ProcessNextFrame");
        long timeoutTime = System.currentTimeMillis() + timeOut;
        myPortal.setProcessorEnabled(this, true);
        startProcessingTime = System.currentTimeMillis();
        reset();

        FrameData imgData = null;
        while( !(waitForDetection ? (imgData != null) : processedFrame.get()) && teamUtil.keepGoing(timeoutTime)){
            teamUtil.pause(30);
            imgData = frameDataQueue.poll();
        }
        if(System.currentTimeMillis()>=timeoutTime){
            teamUtil.log("ProcessNextFrame Timed Out; No detection.");
        }
        if (turnOffProcessor) {
            myPortal.setProcessorEnabled(this, false);
        }
        teamUtil.log("ProcessNextFrame---Finished");
        return imgData;
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
        Rect obscureRect = new Rect(0,OBSCURE_HEIGHT,WIDTH,HEIGHT-OBSCURE_HEIGHT);

        if (undistort) {
            Calib3d.undistort(frame,undistortedMat,cameraMatrix,distCoeffs);
        } else {
            undistortedMat = frame.clone();
        }

        Imgproc.rectangle(undistortedMat, obscureRect, BLACK, -1); // Cover view of robot
        Imgproc.cvtColor(undistortedMat, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise


        switch (targetColor) {
            case YELLOW:
                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, yellowErosionElement);
                break;
            case BLUE:
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, blueErosionElement);
                break;
            case RED:
                Core.inRange(blurredMat, rbyLowHSV, rbyHighHSV, thresholdMatAll); // Get Red and Yellow

                Core.inRange(blurredMat, yellowLowHSV, yellowHighHSV, thresholdMatYellow);
                Core.inRange(blurredMat, blueLowHSV, blueHighHSV, thresholdMatBlue);
                Core.add(thresholdMatYellow, thresholdMatBlue,  thresholdMatYB);
                Imgproc.erode(thresholdMatYB, erodedMat, yellowErosionElement); // Get yellow and blue eroded rects
                Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges
                //thresholdMatYB = thresholdMat.clone();
                thresholdMatYB.setTo(new Scalar(0));
                contours.clear(); // empty the list from last time
                Imgproc.findContours(edgesMat, contours, hierarchyMat, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around yellow/blue rects
                if (!contours.isEmpty()) {
                    MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
                    RotatedRect[] boundRect = new RotatedRect[contours.size()];
                    for (int i = 0; i < contours.size(); i++) {  // for each yellow rect, draw a white rectangle inside it
                        contoursPoly[i] = new MatOfPoint2f();
                        Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                        boundRect[i] = Imgproc.minAreaRect(contoursPoly[i]);
                        Point[] points = new Point[4];
                        boundRect[i].points(points);
                        MatOfPoint r = new MatOfPoint(points);
                        Imgproc.fillConvexPoly(thresholdMatYB, r, new Scalar(255));
                    }
                }
                Core.subtract(thresholdMatAll, thresholdMatYB,  thresholdMat);

                Imgproc.erode(thresholdMat, erodedMat, redErosionElement);

                break;

        }
        Imgproc.rectangle(erodedMat, cropRect, BLACK, 2); // black frame to help with edges and boundaries.

        Imgproc.Canny(erodedMat, edgesMat, 100, 300); // find edges


        List<MatOfPoint> contours = new ArrayList<>();
        contours.clear(); // empty the list from last time
        Imgproc.findContours(erodedMat, contours, hierarchyMat, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE); // find contours around white areas
        RotatedRect[] foundRects = findRectsInPolys(contours);

        if (foundRects.length>0) {
            for (int i = 0; i < foundRects.length; i++) {
                Point endPoint = new Point();
                endPoint.x = (int) (foundRects[i].center.x + 20 * Math.cos(foundRects[i].angle * 3.14 / 180.0));
                endPoint.y =  (int) (foundRects[i].center.y + 20 * Math.sin(foundRects[i].angle * 3.14 / 180.0));
                Point vertices[] = new Point[4];;
                foundRects[i].points(vertices);

            }
        }

        double closestAreaSelection = 1000;
        int closestAreaSelectionNum = -1;
        double xDistCenter;
        double yDistCenter;

        for(int i = 0; i < foundRects.length; i ++){
            // Is the closest one to the target we have seen so far?
            xDistCenter = Math.abs(foundRects[i].center.x - TARGET_X);
            yDistCenter = Math.abs(foundRects[i].center.y - TARGET_Y);
            if (Math.hypot(xDistCenter, yDistCenter) < closestAreaSelection && foundRects[i].size.area() > MIN_AREA_THRESHOLD&&foundRects[i].size.area() < MAX_AREA_THRESHOLD) {
                closestAreaSelection = Math.hypot(xDistCenter, yDistCenter);
                closestAreaSelectionNum = i;
            }
        }


        if (closestAreaSelectionNum == -1) { // nothing big enough
            if (details) teamUtil.log("Saw blobs but nothing big enough");
            foundOne.set(false);
            processedFrame.set(true);
            return null;
        }

        rectArea.set((int)foundRects[closestAreaSelectionNum].size.area());

        //Find the point with the lowest y coordinate
        Point vertices1[] = new Point[4];
        foundRects[closestAreaSelectionNum].points(vertices1); // get the 4 corners of the rotated rectangle
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
                double yDiff = Math.abs(vertices1[lowestPixel].y - vertices1[k].y);
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

        if(foundRects[closestAreaSelectionNum].center.x<FOUND_ONE_LEFT_THRESHOLD||foundRects[closestAreaSelectionNum].center.x>FOUND_ONE_RIGHT_THRESHOLD){
            if (details) {
                teamUtil.log("OPEN CV SAMPLE DETECTOR FOUND BLOCK BUT CENTER X VALUE WAS OUTSIDE MECHANICAL RANGE");
                teamUtil.log("X of Block Center: " + foundRects[closestAreaSelectionNum].center.x);
                teamUtil.log("Y of Block Center: " + foundRects[closestAreaSelectionNum].center.y);
            }
            outsideUseableCameraRange.set(true);
            processedFrame.set(true);
            foundOne.set(false);
        }
        else{
            outsideUseableCameraRange.set(false);
            processedFrame.set(true);

            FrameData imgData = new FrameData();
            imgData.rrect = foundRects[closestAreaSelectionNum];
            imgData.rectAngle = (int) realAngle;
            imgData.rectCenterXOffset = (int) foundRects[closestAreaSelectionNum].center.x - TARGET_X;
            imgData.rectCenterYOffset = -1 * ((int) foundRects[closestAreaSelectionNum].center.y - TARGET_Y);
            imgData.adjRectCenterXOffset=  adjustXCenter((int)foundRects[closestAreaSelectionNum].center.x) - TARGET_X;
            imgData.adjRectCenterYOffset= -1 * (adjustYCenter((int)foundRects[closestAreaSelectionNum].center.y) - TARGET_Y);
            imgData.rectArea = (int) foundRects[closestAreaSelectionNum].size.area();
            imgData.captureTime = captureTimeNanos/1000000;
            imgData.processingTime = System.currentTimeMillis() - startProcessingTime;
            imgData.processingTime2 = System.currentTimeMillis() - imgData.captureTime;

            imgData.lowestPixelX = (int) vertices1[lowestPixel].x;
            imgData.lowestPixelY = (int) vertices1[lowestPixel].y;
            imgData.closestPixelX = (int) vertices1[closestPixel].x;
            imgData.closestPixelY = (int) vertices1[closestPixel].y;

            frameDataQueue.clear(); // Only one object in the queue at a time for now.
            frameDataQueue.add(imgData);
            if (details) teamUtil.log("adding detection to queue. size: " + frameDataQueue.size());

            // old data setting
            rectCenterYOffset.set(-1 * ((int) foundRects[closestAreaSelectionNum].center.y - TARGET_Y));
            rectCenterXOffset.set((int) foundRects[closestAreaSelectionNum].center.x - TARGET_X);
            rectAngle.set((int) realAngle);
            rectArea.set((int) foundRects[closestAreaSelectionNum].size.area());
            foundOne.set(true);

            targetIndex = closestAreaSelectionNum;
            if (details) teamUtil.log("Real Angle" + realAngle + "Lowest: " + vertices1[lowestPixel].x + "," + vertices1[lowestPixel].y+"Closest: " + vertices1[closestPixel].x+ "," +vertices1[closestPixel].y);
        }

        Context context = new Context();
        context.rots = foundRects;
        context.contours = contours;
        return context;
    }

    public RotatedRect[] findRectsInPolys(List <MatOfPoint> contours) {
        RotatedRect[] rects= new RotatedRect[0];
        modPolys = new ArrayList<>(); // empty from previous runs DEBUG ONLY

        if (!contours.isEmpty()) {
            List<MatOfPoint> polygons = new ArrayList<>();
            for (MatOfPoint contour : contours) { // run though the external contours
                double area = Imgproc.contourArea(contour);
                if (area < AREA_THRESHOLD) {
                    System.out.println("Disregarding Contour with area: "+ area);
                    continue;
                } else {
                    System.out.println("Considering Contour with area: "+ area);
                }
                MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                MatOfPoint2f polygon2f = new MatOfPoint2f();
                Imgproc.approxPolyDP(contour2f, polygon2f, polyEp, true); // convert contour to polygon
                RotatedRect[] newRects = findRectsInPoly2(polygon2f);
                rects = mergeRects(rects, newRects);
            }
        }
        return rects;
    }

    public RotatedRect[] findRectsInPoly2 (MatOfPoint2f poly) {
        RotatedRect[] rects= new RotatedRect[0];
        Point[] points = poly.toArray();
        if (points.length<4) {
            System.out.println("Disregarding polygon with less than 4 points");
            return rects; // no rectangles in this polygon
        }

        System.out.println("==========================Analyizing Poly.  Points: "+ points.length);
        points = removeShortSegments(points); // remove all the short segments
        points = combineStraightLineSegments(points); // combine segments that form a straight line
        for (int p1 = 0; p1 < points.length;p1++) {
            System.out.println("---- Starting at "+p1);
            int p2 = (p1+1)%points.length;
            int p3 = (p2+1)%points.length;

            // We now have P1, P2, and P3 which form 2 edges of a hypothetical target rectangle
            // So now we check to see it passes the tests

            double slope1, slope2;
            slope1 = calculateSlope(points[p1], points[p2]);
            slope2 = calculateSlope(points[p2], points[p3]);
            double l1 = findLength(points[p1], points[p2]);
            double l2 = findLength(points[p2], points[p3]);

            System.out.println("--");
            System.out.println("Looking at points: "+ p1+ points[p1] + ", " + p2+ points[p2] + ", " + p3+ points[p3]);
            System.out.println(String.format("s1: %.1f s2: %.1f L1: %.1f L2: %.0f", slope1, slope2, l1, l2));

            // Start by looking for an approximately 90 degree angle
            if (!isRightAngle(slope1, slope2, RIGHT_ANGLE_TOLERANCE)) {
                // two lines didn't form a right angle
                System.out.println("Disregarding candidate with non-right angle");
                continue;
            }

            // now check lengths and adjust
            if (Math.abs(l1-LONG_LENGTH_TARGET)<LONG_LENGTH_THRESHOLD && Math.abs(l2-SHORT_LENGTH_TARGET)<SHORT_LENGTH_THRESHOLD) {
                System.out.println("Lengths OK-L1 long side");
                // adjust p1 to match long target?
                // adjust p3 to match short target?
            } else if (Math.abs(l2-LONG_LENGTH_TARGET)<LONG_LENGTH_THRESHOLD && Math.abs(l1-SHORT_LENGTH_TARGET)<SHORT_LENGTH_THRESHOLD) {
                System.out.println("Lengths OK-L2 long side");
                // adjust p1 to match short target?
                // adjust p3 to match long target?
            } else {
                // lengths not close enough so move on
                System.out.println("Disregarding candidate with wrong lengths");
                continue;
            }

            // We found one, so impute point 4
            Point point4 = findFourthPoint(points[p1], points[p2], points[p3]);
            System.out.println("Computing 4th point: "+ point4);
            // Test to make sure it is inside the polygon
            double result = Imgproc.pointPolygonTest(poly, point4, true );
            if (result+EXTERNAL_THRESHOLD < 0) {
                // imputed 4th point too far outside polygon (we are in a convex portion)
                System.out.println("Disregarding candidate with 4th point outside polygon: "+result);
                continue;
            } else {
                System.out.println("4th point inside (or close to) polygon: "+result);
            }

            // It all looks good so lets set up the found rect and subtract it from remaining poly
            RotatedRect foundRect = getRotatedRect(points[p1], points[p2], points[p3], point4);
            RotatedRect[] updatedRects = new RotatedRect[rects.length + 1];
            System.arraycopy(rects, 0, updatedRects, 0, rects.length);
            updatedRects[rects.length] = foundRect;
            rects = updatedRects;

            System.out.println("||||||||||||||||||||||||Found One-------------------------------------------------");

            points[p2].x = point4.x;
            points[p2].y = point4.y;
            poly = new MatOfPoint2f(points); // update poly contour for next time through loop


        }
        modPolys.add(points.clone());  // DEBUG only stash a copy of the modified version of the polygon for viewing
        return rects;
    }

    public RotatedRect getRotatedRect(Point p1, Point p2, Point p3, Point p4) {
        Point[] points = new Point[4];
        points[0] = p1;
        points[1] = p2;
        points[2] = p3;
        points[3] = p4;

        // Create a MatOfPoint2f from the points
        MatOfPoint2f matOfPoints = new MatOfPoint2f(points);

        // Calculate the rotated rectangle
        return ( Imgproc.minAreaRect(matOfPoints));
    }


    public Point[] combineStraightLineSegments(Point[] polygon) {
        if (polygon.length < 3) {
            return polygon;
        }
        ArrayList<Point> updatedPolygon = new ArrayList<>();
        int n = polygon.length;
        int i = 0;
        while (i < n) {
            Point current = polygon[i];
            Point next = polygon[(i + 1) % n]; // Wrap around to form a closed polygon

            // Always add the current point
            updatedPolygon.add(current);

            // Check if the next segment forms part of a straight line
            while (isStraightLine(current, next, polygon[(i + 2) % n])) {
                System.out.println("Disregarding point on straight line "+((i + 1) % n)+polygon[(i + 1) % n]);
                i++;
                next = polygon[(i + 2) % n]; // Extend the sequence to the next segment
            }

            // Move to the next unique segment
            i++;
        }
        return updatedPolygon.toArray(new Point[0]);
    }


    public  Point[] removeShortSegments(Point[] polygon) {
        if (polygon.length < 3) {
            return polygon;
        }

        ArrayList<Point> updatedPolygon = new ArrayList<>();
        int n = polygon.length;

        int i = 0;
        while (i < n) {
            Point current = polygon[i];
            Point next = polygon[(i + 1) % n]; // Wrap around to form a closed polygon

            double distance = findLength(current, next);

            if (distance >= SEGMENT_THRESHOLD) {
                // Keep this point in the updated polygon
                updatedPolygon.add(current);
                i++; // Move to the next segment
            } else {
                System.out.println("Disregarding short segment between "+i+polygon[i]+" and "+(i+1)+polygon[(i + 1) % n]+ " with length: "+distance);

                // Start collecting a sequence of short segments
                int start = i; // Start of the short segment sequence
                distance = findLength(polygon[(i + 1) % n], polygon[(i + 2) % n]);
                while (distance < SEGMENT_THRESHOLD) {
                    System.out.println("Disregarding segment between "+(i + 1)+polygon[(i + 1) % n]+" and "+(i+2)+polygon[(i + 2) % n]+ " with length: "+distance);
                    i++;
                    distance = findLength(polygon[(i + 1) % n], polygon[(i + 2) % n]);
                }
                // Add a single midpoint between the points before and after the sequence
                Point before = polygon[start];
                Point after = polygon[(i + 2) % n];
                Point midpoint = calculateMidpoint(before, after);
                updatedPolygon.add(midpoint);

                // Skip over the sequence of short segments
                i += 2;
            }
        }

        return updatedPolygon.toArray(new Point[0]);
    }

    private boolean isStraightLine(Point p1, Point p2, Point p3) {
        double slope1 = calculateSlope(p1, p2);
        double slope2 = calculateSlope(p2, p3);

        // If either slope is undefined (vertical line), handle separately
        if (Double.isInfinite(slope1) && Double.isInfinite(slope2)) {
            return true; // Both are vertical
        } else if (Double.isInfinite(slope1) || Double.isInfinite(slope2)) {
            return false; // One is vertical, the other is not
        }

        // Calculate the angular difference
        double angle1 = Math.toDegrees(Math.atan(slope1));
        double angle2 = Math.toDegrees(Math.atan(slope2));
        double angleDifference = Math.abs(angle1 - angle2);

        // Normalize the angle difference to [0, 180]
        angleDifference = Math.min(angleDifference, 180 - angleDifference);

        // Check if the angular difference is within the threshold
        return angleDifference <= STRAIGHT_THRESHOLD;
    }

    private static double calculateSlope(Point p1, Point p2) {
        if (p1.x == p2.x) {
            return Double.POSITIVE_INFINITY; // Vertical line
        }
        return (double) (p2.y - p1.y) / (p2.x - p1.x);
    }

    private static Point calculateMidpoint(Point p1, Point p2) {
        double midX = (p1.x + p2.x) / 2f;
        double midY = (p1.y + p2.y) / 2f;
        return new Point(midX, midY);
    }

    public Point findFourthPoint (Point p1, Point p2, Point p3) {
        double x4 = p1.x + p3.x - p2.x;
        double y4 = p1.y + p3.y - p2.y;
        return new Point(x4, y4);
    }

    public boolean isRightAngle(double s1, double s2, double maxDeviationDegrees) {
        double angle = Math.toDegrees(Math.atan(Math.abs((s1 - s2) / (1 + s1 * s2))));
        System.out.println("Checking Angle: "+ angle);
        return Math.abs(90 - angle) <= maxDeviationDegrees;
    }

    public double findLength(Point a, Point b) {
        return Math.sqrt(Math.pow(b.y-a.y,2)+Math.pow(b.x-a.x,2));
    }

    public static RotatedRect[] mergeRects(RotatedRect[] a, RotatedRect[] b) {
        RotatedRect[] result = new RotatedRect[a.length+b.length];
        System.arraycopy(a, 0, result, 0, a.length);
        System.arraycopy(b, 0, result, a.length, b.length);
        return result;
    }




    public int adjustYCenter(int y) {
        if (TARGET_Y < HEIGHT/2) { // uniform distortion to the top, but varying to the bottom as we cross the center
            if (y < TARGET_Y) {
                return (int) (TARGET_Y - (TARGET_Y-y) * CAM_ANGLE_ADJUST_NEG_Y); // assumes linear from target but it really is linear from center of cam (might need a "b" coeffecient)
            } else if (y > TARGET_Y) {
                if (y < HEIGHT/2) { // between target and center of camera (as we move away from target we are approaching the center of the camera so distances get less distorted)
                    return (int) ((y-TARGET_Y) * CAM_ANGLE_ADJUST_MID_Y + TARGET_Y);
                } else { // below target and below center of camera (as we move away from center of camera, distances get more distorted
                    return (int) ((y-HEIGHT/2) * CAM_ANGLE_ADJUST_POS_Y + HEIGHT/2+ CAM_ANGLE_ADJUST_POS_Y_B); // linear from center of cam, so needed a "b" coefficient to deal with distance from target to center of cam
                }
            } else { // already over the target
                return y;
            }
        } else {
            teamUtil.log("adjustYTarget Case Not Implemented!");
            return y;
        }
    }

    public int adjustXCenter(int x) {
        if (TARGET_X > WIDTH/2) { // // uniform distortion to the right, but varying to the left as we cross the center
            if (x > TARGET_X) {
                return (int) (TARGET_X + (x-TARGET_X) * CAM_ANGLE_ADJUST_POS_X); // assumes linear from target but it really is linear from center of cam (might need a "b" coeffecient)
            } else if (x < TARGET_X) {
                if (x > WIDTH/2) { // between target and center of camera (as we move away from target we are approaching the center of the camera so distances get less distorted)
                    return (int) (TARGET_X - (TARGET_X-x) * CAM_ANGLE_ADJUST_MID_X);
                } else { // left of target and left of center of camera (as we move away from center of camera, distances get more distorted
                    return (int) ( WIDTH/2 - (WIDTH/2-x) * CAM_ANGLE_ADJUST_NEG_X + CAM_ANGLE_ADJUST_NEG_X_B); // linear from center of cam, so needed a "b" coefficient to deal with distance from target to center of cam
                }
            } else { // already over the target
                return x;
            }
        } else {
            teamUtil.log("adjustXTarget Case Not Implemented!");
            return x;
        }
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
                case UNDISTORTED: { Utils.matToBitmap(undistortedMat,bmp); break;}
                case HSV: { Utils.matToBitmap(HSVMat, bmp); break; }
                case BLURRED: { Utils.matToBitmap(blurredMat, bmp); break;}
                case THRESHOLD: { Utils.matToBitmap(thresholdMat, bmp); break;}
                case ERODED: { Utils.matToBitmap(erodedMat, bmp); break;}
                case EDGES: { Utils.matToBitmap(edgesMat, bmp); break;}
                default: {/*RAW image is already on canvas*/}
            }
            Bitmap resizedBitmap = Bitmap.createScaledBitmap(bmp, (int)(WIDTH*scaleBmpPxToCanvasPx), (int)(HEIGHT*scaleBmpPxToCanvasPx), false);
            canvas.drawBitmap(resizedBitmap, 0,0,null);
        }


        Paint samplePaint = new Paint();
        samplePaint.setColor(Color.GREEN);
        samplePaint.setStyle(Paint.Style.STROKE);
        samplePaint.setStrokeWidth(scaleCanvasDensity * 4);
        canvas.drawCircle(WIDTH/2*scaleBmpPxToCanvasPx, HEIGHT/2*scaleBmpPxToCanvasPx, GOLDILOCKS_ZONE_RADIUS,samplePaint);


        canvas.drawCircle((float)TARGET_X*scaleBmpPxToCanvasPx, (float)TARGET_Y*scaleBmpPxToCanvasPx, 10,samplePaint);
        canvas.drawRect((float)sampleRect.tl().x*scaleBmpPxToCanvasPx, (float)sampleRect.tl().y*scaleBmpPxToCanvasPx, (float)sampleRect.br().x*scaleBmpPxToCanvasPx, (float)sampleRect.br().y*scaleBmpPxToCanvasPx,samplePaint);

        if (userContext != null) {
            RotatedRect[] rotatedRect = ((Context) userContext).rots;
            if (rotatedRect != null) {
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
            List<MatOfPoint> drawContours = ((Context) userContext).contours;
            if (drawContours != null) {
                if (!drawContours.isEmpty()) {
                    Paint polyPaint = new Paint();
                    polyPaint.setColor(Color.WHITE);
                    polyPaint.setStyle(Paint.Style.STROKE);
                    polyPaint.setStrokeWidth(scaleCanvasDensity * 6);

                    List<MatOfPoint> polygons = new ArrayList<>();
                    for (MatOfPoint contour : drawContours) {
                        MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
                        MatOfPoint2f polygon2f = new MatOfPoint2f();
                        //Imgproc.approxPolyDP(contour2f, polygon2f, 0.02 * Imgproc.arcLength(contour2f, true), true);
                        Imgproc.approxPolyDP(contour2f, polygon2f, 3, true);
                        Point[] points = polygon2f.toArray();
                        for (int i = 0; i < points.length - 1; i++) {
                            canvas.drawLine((float) points[i].x * scaleBmpPxToCanvasPx, (float) points[i].y * scaleBmpPxToCanvasPx, (float) points[i + 1].x * scaleBmpPxToCanvasPx, (float) points[i + 1].y * scaleBmpPxToCanvasPx, polyPaint);
                        }
                        canvas.drawLine((float) points[0].x * scaleBmpPxToCanvasPx, (float) points[0].y * scaleBmpPxToCanvasPx, (float) points[points.length-1].x * scaleBmpPxToCanvasPx, (float) points[points.length-1].y * scaleBmpPxToCanvasPx, polyPaint);
                    }
                }
            }

        }
        // TODO  figure out how to save a processed frame (Android canvas) to the local storage (SDCard)
        // TODO: set up an Atomic boolean to trigger this behavior for a single frame?
    }


}