package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.RotatedRect;
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
import java.util.Vector;

public class MaskByColor extends OpenCvPipeline{
    public static int choice = 0;
    public static int erosionSize = 5;
    public Mat rotatedRectMat = new Mat();

    public Mat grayMat = new Mat();
    public Mat noCornersBg = new Mat();

    public double sampleWidth = 2.5;
    public static double width2pixMult = 13.5;
    public double sampleHeight = 3.5;
    public double width = 320;
    public double height = 240;
    public Mat endMat = new Mat();
    public double avgContourArea = 500 * width/320 * height/240;

    public Mat erodeKernal;
    private Scalar black = new Scalar(0,0,0);
    public Mat cornersMat = new Mat();
    public Mat cornersMask = new Mat();

    public Mat invCornersMask = new Mat();

    public double defaultRectProportion = 2.3;
    public ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();

    public double rectProportion;
    public ArrayList<Double> anglesArr = new ArrayList<Double>();

    public Size rectSize = new Size();
    public static double contourSimplification = 0.02;
    public Mat tempMat2 = new Mat();
    public Scalar lowerGray = new Scalar(0,0,0);
    public Scalar upperGray = new Scalar(0,0,0);
    private Scalar lowerYellow = new Scalar(12.8, 80.8, 131.8);
    private Scalar upperYellow = new Scalar(45.3, 255, 255);
    private Scalar lowerBlue = new Scalar(92.1, 24.1, 68.0);
    private Scalar upperBlue = new Scalar(161.7, 255, 255);
    private Scalar lowerRed = new Scalar(145.9, 185.4, 51);
    private Scalar upperRed = new Scalar(228, 242.3, 255);
    private Scalar lowerRed2 = new Scalar(0, 131.8, 79.3);
    private Scalar upperRed2 = new Scalar(5.7, 255, 255);
    public Mat hsvMask = new Mat();
    public Mat tempMat = new Mat();
    public Mat harrisMask = new Mat();
    public static double angleFlipper = 1.0;
    public static int morph = 5;
    public static int size = 3;
    //public static double rectangleMarginOfError = 2.5;
    public List<MatOfPoint> contours;
    public Mat normalizedMat = new Mat();

    public static int blocksizeInt = 3;
    public static int harrisKSize = 3;

    public static double harrisK = 0.00;

    public Mat harrisMat = new Mat();

    public static double contourArea = 0;



    public Mat rgb2hsv(Mat source){
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2HSV);
        return tempMat;
    }
    public Mat rgb2lab(Mat source){
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2Lab);
        return tempMat;
    }

    public Mat rgb2gray (Mat source){
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2GRAY);
        return tempMat;
    }
    Telemetry telemetry;

    public MaskByColor(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public Mat highLightCorners(Mat hsvMat, int blocksizeInt, int harrisKSize,double harrisK){
        Imgproc.cornerHarris(hsvMask, harrisMat, blocksizeInt, harrisKSize, harrisK);
        Core.normalize(harrisMat, harrisMat, 0, 255, Core.NORM_MINMAX);
        harrisMat.convertTo(harrisMat, CvType.CV_8U);
        return harrisMat;
    }
    @Override
    public void init(Mat firstFrame)
    {
        rgb2hsv(firstFrame);
    }
    public Mat blurMask(Mat input){
        Size gaussainSize = new Size(size, size);
        Imgproc.GaussianBlur(input, tempMat, gaussainSize, 1.0);
        Core.inRange(tempMat, black, black, tempMat);
        Core.bitwise_not(tempMat, tempMat);
        return tempMat;
    }

    public Mat removeNoise(Mat input){
        Mat morphKernal = new Mat(morph, morph, 0);
        tempMat = blurMask(input);
        Imgproc.morphologyEx(tempMat, tempMat, Imgproc.MORPH_OPEN, morphKernal);
        return tempMat;
    }

    public List<MatOfPoint> getContours(Mat input){
        Mat hierarchy = new Mat();
        List<MatOfPoint> contoursArr = new ArrayList<>();
        Imgproc.findContours(input, contoursArr, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contoursArr;
    }

    public ArrayList<Point> getCenterPoint(List<MatOfPoint> contoursArr){
        ArrayList<Point> output = new ArrayList<Point>();
        for (MatOfPoint contour: contoursArr){
            Point point = new Point();
            Moments moments = Imgproc.moments(contour);
            //uses the moments to get the center of any object
            point.x = (int)moments.get_m10() / (int)moments.get_m00();
            point.y = (int)moments.get_m01() / (int)moments.get_m00();
            output.add(point);
        }
        return output;
    }

    public Mat drawRotatedRectange(Mat input, RotatedRect rotatedRect){
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);

        // Draw the rotated rectangle on the image
        for (int i = 0; i < 4; i++) {
            Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
        }
        return input;
    }

    public List<MatOfPoint> simplifyContours(List<MatOfPoint> contours){
        double epsilon;
        for (int i = 0; i < contours.size(); i++){
            MatOfPoint contour = contours.get(i);
            MatOfPoint outputMat = new MatOfPoint();
            MatOfPoint2f newMtx = new MatOfPoint2f(contour.toArray());
            epsilon = contourSimplification * Imgproc.arcLength(newMtx, true);
            MatOfPoint2f outputMtx = new MatOfPoint2f();
            Imgproc.approxPolyDP(newMtx, outputMtx, epsilon, true);
            outputMtx.convertTo(outputMat, CvType.CV_32S);
            contours.set(i, outputMat);

        }
        return contours;
    }

    public boolean inRange(double input, double min, double max){
        return (input >= min && input <= max);
    }


    public int getNumberOfBlocks(MatOfPoint contour){
        return (int)(Imgproc.contourArea(contour) /avgContourArea);
    }



    @Override
    public Mat processFrame(Mat input){
        Imgproc.resize(input, input, new Size(320,240));

        Mat hsvMat = rgb2hsv(input);
        if (choice == 0){
            Core.inRange(hsvMat, lowerRed, upperRed, tempMat);
            Core.inRange(hsvMat, lowerRed2, upperRed2, tempMat2);
            Core.bitwise_or(tempMat, tempMat2, hsvMask);
        }
        else if (choice == 1){
            Core.inRange(hsvMat, lowerBlue, upperBlue, hsvMask);
        }
        else if (choice == 2){
            Core.inRange(hsvMat, lowerYellow, upperYellow, hsvMask);
        }
        else{
            Core.inRange(hsvMat, lowerGray, upperGray, hsvMask);
        }


        //filteredContours.clear();
        //anglesArr.clear();
        //Imgproc.cvtColor(input, hsvMask, Imgproc.COLOR_RGB2GRAY);





        //adds rectangles and filters the contour list
        //for (MatOfPoint contour: contours){
        //    MatOfPoint2f newMtx = new MatOfPoint2f(contour.toArray());
        //    RotatedRect rect = Imgproc.minAreaRect(newMtx);
        //    rectSize = rect.size;
        //    rectProportion = rectSize.width/rectSize.height;
        //check to see if dimensions match the data and filter the data based on that
        //    if ((rectProportion >= defaultRectProportion - rectangleMarginOfError && rectProportion <= defaultRectProportion + rectangleMarginOfError) || (1/rectProportion >= defaultRectProportion - rectangleMarginOfError && 1/rectProportion <= defaultRectProportion + rectProportion)) {
        //        input = drawRotatedRectange(input, rect);
        //        anglesArr.add(rect.angle);
        //        filteredContours.add(contour);
        //    }
        //}
        cornersMask = new Mat();
        hsvMask = removeNoise(hsvMask);
        erodeKernal = new Mat(erosionSize, erosionSize, 0);
        Imgproc.erode(hsvMask,hsvMask, erodeKernal);

        grayMat = rgb2gray(input);
        cornersMat = highLightCorners(grayMat, blocksizeInt, harrisKSize, harrisK);

        Core.bitwise_or(grayMat, grayMat, cornersMask,hsvMask);
        Core.normalize(cornersMask, normalizedMat, 0, 255, Core.NORM_MINMAX);
        if(choice == 0 ){
            upperGray.val[0] = 140;

        }
        else if (choice == 1){
            upperGray.val[0] = 105;
        }
        else if(choice == 2){
            upperGray.val[0] = 205;
        }
        Core.inRange(normalizedMat, lowerGray, upperGray, endMat);
        //red 0-160
        //blue 0-111
        //yellow -200
        //Imgproc.erode(endMat,endMat, erodeKernal);
        Imgproc.dilate(endMat, endMat, erodeKernal);
        Imgproc.erode(endMat, endMat, erodeKernal);
        Core.bitwise_not(endMat, endMat);
        contours = getContours(endMat);
        contours = simplifyContours(contours);
        for (MatOfPoint contour: contours){
            RotatedRect rect = new RotatedRect();
            rect = Imgproc.minAreaRect(new MatOfPoint2f(contour.toArray()));
            input = drawRotatedRectange(input, rect);
        }

        //Core.inRange(cornersMat, new Scalar(0,0,0), new Scalar(1,1,1), noCornersBg);
        ArrayList<Point> centerPoints = getCenterPoint(contours);
        //Imgproc.drawContours(input, contours, -1, new Scalar(255,255,0),3);


        for (Point point : centerPoints){
            Imgproc.circle(input, point, 2, new Scalar(255,0,0), 5);
        }
        telemetry.update();
        return input;

    }



}
