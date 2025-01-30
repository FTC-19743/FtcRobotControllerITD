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
public class OpenCVSampleDetectorBlockChecker extends OpenCVProcesser {
    HardwareMap hardwareMap;
    Telemetry telemetry;



    Scalar BLACK = new Scalar(0, 0, 0);
    Scalar GREEN = new Scalar(0, 255, 0);


    static public final int WIDTH = 640;
    static public final int HEIGHT = 480;
    static public int OBSCURE_HEIGHT = 460;
    Rect checkArea = new Rect(0, 0, WIDTH, HEIGHT);



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

    public OpenCVSampleDetectorBlockChecker() {
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.telemetry;
    }
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
        teamUtil.log("Initializing OpenCVSampleDetectorV2 processor");
        for (int i=0;i<3;i++)
            cameraMatrix.put(i,0, cameraMatrixData[i]); // preload calibration data
        distCoeffs.put(0, 0, distCoeffsData);

        teamUtil.log("Initializing OpenCVSampleDetectorV2 processor - FINISHED");
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


        if (undistort) {
            Calib3d.undistort(frame,undistortedMat,cameraMatrix,distCoeffs);
        } else {
            undistortedMat = frame.clone();
        }


        Imgproc.cvtColor(undistortedMat, HSVMat, Imgproc.COLOR_RGB2HSV); // convert to HSV
        Imgproc.blur(HSVMat, blurredMat, blurFactorSize); // get rid of noise







        Context context = new Context();
        context.contours = contours;
        return context;


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