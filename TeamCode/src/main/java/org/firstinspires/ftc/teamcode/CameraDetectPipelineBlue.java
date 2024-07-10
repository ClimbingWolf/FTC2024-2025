package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.imgproc.Moments;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;

public class CameraDetectPipelineBlue extends OpenCvPipeline{
    public double nonZero = 0;
    public String position = "";
    public Scalar rectRed = new Scalar (255,100,100);

    public Scalar rectGreen = new Scalar(120, 255, 255);
    public Scalar blueLower = new Scalar (85,128,80);

    public Scalar blueUpper = new Scalar (134, 255, 255);


    public static boolean mask = false;

    public Point p1r1= new Point(0,160);
    public Point p2r1 = new Point(100,260);
    public Point p1r2= new Point(270,160);
    public Point p2r2 = new Point(370,260);
    public Point p1r3= new Point(530,160);
    public Point p2r3 = new Point(630,260);

    public Rect rectangle;
    public Rect rectangle2;
    public Rect rectangle3;
    private double rect1Count = 0;
    private double rect2Count= 0;
    private double rect3Count = 0;

    public Mat blueMask1 = new Mat();
    Mat submat = new Mat();
    public Mat rectSubmat(Mat source, Point point1, Point point2){
        Rect rectangle = new Rect(point1, point2);
        return source.submat(rectangle);
    }

    public Mat rectSubmat(Mat source, Rect rectangle){
        return source.submat(rectangle);
    }

    public Mat rgb2hsv(Mat source){
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2HSV);
        return tempMat;
    }
    @Override
    public void init(Mat firstFrame)
    {
        rgb2hsv(firstFrame);
    }

    public double countNonZeroInRect(Mat input, Rect rectangle){
        submat = rectSubmat(input, rectangle);
        submat = rgb2hsv(submat);
        Core.inRange(submat, blueLower, blueUpper, blueMask1);
        return Core.countNonZero(blueMask1);
    }
    @Override
    public Mat processFrame(Mat input){
        rectangle = new Rect(p1r1, p2r1);
        rectangle2 = new Rect(p1r2, p2r2);
        rectangle3 = new Rect(p1r3,p2r3);
        rect1Count = countNonZeroInRect(input, rectangle);
        rect2Count = countNonZeroInRect(input, rectangle2);
        rect3Count = countNonZeroInRect(input, rectangle3);

        if(rect1Count > rect2Count && rect1Count > rect3Count){
            Imgproc.rectangle(input, rectangle, rectGreen, 5);
            Imgproc.rectangle(input, rectangle2, rectRed, 5);
            Imgproc.rectangle(input, rectangle3, rectRed, 5);
            position = "left";
        }
        else if (rect2Count > rect1Count && rect2Count > rect3Count){
            Imgproc.rectangle(input, rectangle, rectRed, 5);
            Imgproc.rectangle(input, rectangle2, rectGreen, 5);
            Imgproc.rectangle(input, rectangle3, rectRed, 5);
            position = "middle";
        }
        else{
            Imgproc.rectangle(input, rectangle, rectRed, 5);
            Imgproc.rectangle(input, rectangle2, rectRed, 5);
            Imgproc.rectangle(input, rectangle3, rectGreen, 5);
            position = "right";
        }

        return input;
    }


}
