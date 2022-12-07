package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class PwrDeterminationPipeline extends OpenCvPipeline {
    public enum PwrPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    static Scalar BLUE = new Scalar(0, 0, 255);
    static Scalar GREEN = new Scalar(0, 255, 0);

    static Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(181, 48);
    static Point REGION2_TOPLEFT_ANCHOR_POINT = new Point(181, 98);
    static Point REGION3_TOPLEFT_ANCHOR_POINT = new Point(181, 148);
    static int REGION_WIDTH = 20;
    static int REGION_HEIGHT = 20;

    /*
     * Points which actually define the sample region rectangles, derived from above values
     *
     * Example of how points A and B work to define a rectangle
     *
     *   ------------------------------------
     *   | (0,0) Point A                    |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                                  |
     *   |                  Point B (70,50) |
     *   ------------------------------------
     *
     */
    Point region1_pointA = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x,
            REGION1_TOPLEFT_ANCHOR_POINT.y);
    Point region1_pointB = new Point(
            REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region2_pointA = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x,
            REGION2_TOPLEFT_ANCHOR_POINT.y);
    Point region2_pointB = new Point(
            REGION2_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION2_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);
    Point region3_pointA = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x,
            REGION3_TOPLEFT_ANCHOR_POINT.y);
    Point region3_pointB = new Point(
            REGION3_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            REGION3_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    Mat region1_Cb, region2_Cb, region3_Cb;
    Mat YCrCb = new Mat();
    Mat Cb = new Mat();
    int avg1, avg2, avg3;

    // Volatile since accessed by OpMode thread w/o synchronization
    private volatile PwrPosition position = PwrPosition.LEFT;

    void inputToCb(Mat input) {
        Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
        Core.extractChannel(YCrCb, Cb, 2);
    }

    @Override
    public void init(Mat firstFrame) {
        inputToCb(firstFrame);

        region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
        region2_Cb = Cb.submat(new Rect(region2_pointA, region2_pointB));
        region3_Cb = Cb.submat(new Rect(region3_pointA, region3_pointB));
    }

    @Override
    public Mat processFrame(Mat input) {
        inputToCb(input);

        avg1 = (int) Core.mean(region1_Cb).val[0];
        avg2 = (int) Core.mean(region2_Cb).val[0];
        avg3 = (int) Core.mean(region3_Cb).val[0];
        Imgproc.rectangle(input, region1_pointA, region1_pointB, BLUE, 2);
        Imgproc.rectangle(input, region2_pointA, region2_pointB, BLUE, 2);
        Imgproc.rectangle(input, region3_pointA, region3_pointB, BLUE, 2);

        int max = Math.max(Math.max(avg1, avg2), avg3);

        if (max == avg1) {
            position = PwrPosition.LEFT;
            Imgproc.rectangle(input, region1_pointA, region1_pointB, GREEN, -1);
        } else if (max == avg2) {
            position = PwrPosition.CENTER;
            Imgproc.rectangle(input, region2_pointA, region2_pointB, GREEN, -1);
        } else {
            position = PwrPosition.RIGHT;
            Imgproc.rectangle(input, region3_pointA, region3_pointB, GREEN, -1);
        }

        return input;
    }

    public PwrPosition getAnalysis() {
        return position;
    }
}
