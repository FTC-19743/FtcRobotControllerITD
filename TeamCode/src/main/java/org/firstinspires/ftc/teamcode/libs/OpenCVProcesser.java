package org.firstinspires.ftc.teamcode.libs;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

public abstract class OpenCVProcesser implements VisionProcessor {

    Mat submat = new Mat();



    // Convert from OpenCV Rect to Android Graphics Rect.  Useful for writing on the Android Canvas in onDrawFrame
    public android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);
        return new android.graphics.Rect(left, top, right, bottom);
    }

    // Compute the average saturation in a given rectangle on an HSV mat
    // Useful for finding something against the grey background of a FTC mat!
    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    protected double getAvgValue(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[2];
    }

    protected Scalar getAverages (Mat input, Rect rect) {
        submat = input.submat(rect);
        return(Core.mean(submat));
    }



}
