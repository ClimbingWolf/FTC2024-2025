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
public class MaskByColor extends OpenCvPipeline{
    public static int choice = 0;
    public static Scalar lowerHsv = new Scalar(0,0,0);
    public static Scalar upperHsv = new Scalar(0,0,0);
    private Scalar lowerYellow = new Scalar(14.2, 178, 240.8);
    private Scalar upperYellow = new Scalar(28.3, 233.8, 255);
    private Scalar lowerBlue = new Scalar(92.1, 24.1, 68.0);
    private Scalar upperBlue = new Scalar(141.7, 195.5, 242.3);
    private Scalar lowerRed = new Scalar(148.8, 92.1, 0.0);
    private Scalar upperRed = new Scalar(255, 221.0, 255.0);
    private Scalar lowerRed2 = new Scalar(0, 131.8, 79.3);
    private Scalar upperRed2 = new Scalar(5.7, 208.3, 255);
    public Mat hsvMask = new Mat();
    public Mat tempMat = new Mat();

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

    @Override
    public Mat processFrame(Mat input){
        Mat hsvMat = rgb2hsv(input);
        if (choice == 0){
            Core.inRange(hsvMat, lowerRed, upperRed, hsvMask);
            Core.inRange(hsvMat, lowerRed2, upperRed2, tempMat);
            Core.bitwise_or(hsvMask, tempMat, hsvMask);
        }
        else if (choice == 1){
            Core.inRange(hsvMat, lowerBlue, upperBlue, hsvMask);
        }
        else if (choice == 2){
            Core.inRange(hsvMat, lowerYellow, upperYellow, hsvMask);
        }
        else{
            Core.inRange(hsvMat, lowerHsv, upperHsv, hsvMask);
        }
        Core.bitwise_and(input, input, input);
        return hsvMask;
    }


}
