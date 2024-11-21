package org.firstinspires.ftc.teamcode.writtencode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.geometry.euclidean.twod.Vector2D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point3;
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
import java.util.Arrays;
import java.util.List;
import java.util.Vector;

public class GetColorMaskPoints extends OpenCvPipeline {
    public int choice = 0;


    public static int erosionSize = 7;

    private int dilusionSize = 0;
    public Mat rotatedRectMat = new Mat();

    public Mat grayMat = new Mat();
    public Mat noCornersBg = new Mat();

    public double sampleWidth = 2.5;

    private double width2pixMult = 13.5;
    public double sampleHeight = 3.5;
    public double width = 320;
    public double height = width/16 * 9;
    public Mat endMat = new Mat();

    public Point realPoint = new Point();
    public double avgContourArea = 500 * width / 320 * height / 240;

    public Mat erodeKernal;
    public Mat diluteKernal;

    public double focalLen = 0.1444882;
    private Scalar black = new Scalar(0, 0, 0);
    public Mat cornersMat = new Mat();
    public Mat cornersMask = new Mat();

    public Mat invCornersMask = new Mat();

    public double defaultRectProportion = 2.3;
    public ArrayList<MatOfPoint> filteredContours = new ArrayList<MatOfPoint>();

    public double rectProportion;
    public ArrayList<Double> anglesArr = new ArrayList<Double>();

    public Size rectSize = new Size();

    public MatOfPoint epicContour = new MatOfPoint();
    private double contourSimplification = 0.02;
    public Mat tempMat2 = new Mat();
    public Scalar lowerGray = new Scalar(0, 0, 0);
    public Scalar upperGray = new Scalar(0, 0, 0);
    private Scalar lowerYellow = new Scalar(1.4, 168.6, 94.9);
    private Scalar upperYellow = new Scalar(28.3, 240.8, 249.3);
    private Scalar lowerBlue = new Scalar(96.3, 155.8, 60.9);
    private Scalar upperBlue = new Scalar(119, 255, 246.5);
    private Scalar lowerRed = new Scalar(137.4, 130.3, 26.9);
    private Scalar upperRed = new Scalar(255, 255, 233.8);
    private Scalar lowerRed2 = new Scalar(0, 131.8, 79.3);
    private Scalar upperRed2 = new Scalar(5.7, 255, 255);
    public Mat hsvMask = new Mat();
    public Mat tempMat = new Mat();
    public Mat harrisMask = new Mat();
    private double angleFlipper = 1.0;

    public static double startIn = 5;
    public static double endIn = 15;
    private int morph = 5;
    private int size = 3;
    //private double rectangleMarginOfError = 2.5;
    public List<MatOfPoint> contours;
    public Mat normalizedMat = new Mat();

    private int blocksizeInt = 3;

    public static double xConst4Cam = 1178.57;
    public static double yConst4Cam = 0.53;

    private double camYReal = 12;
    private double camXReal = 12;
    private int harrisKSize = 3;

    private double camAngleDeg = 45;

    private double fovHDeg = 70.42;

    public double zReal = -12.5;

    private double fovVDeg = 43.3;

    private double fovDiag = 78;

    private double harrisK = 0.00;

    public Mat harrisMat = new Mat();

    private double contourArea = 0;

    public static double maxRange = 15;
    public static double minRange = 5;

    public Point closestPoint = new Point();
    public Point startPoint = new Point(width/2, height-10);

    public List<MatOfPoint> matList = Arrays.asList(new MatOfPoint());


    public Mat rgb2hsv(Mat source) {
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2HSV);
        return tempMat;
    }

    private double rotateX(double x, double y, double thetaRadians){
        return x*Math.cos(thetaRadians) - y*Math.sin(thetaRadians);
    }
    private double rotateY(double x, double y, double thetaRadians){
        return x*Math.sin(thetaRadians) + y*Math.cos(thetaRadians);
    }

    public Mat rgb2lab(Mat source) {
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2Lab);
        return tempMat;
    }

    public Mat rgb2gray(Mat source) {
        Mat tempMat = new Mat();
        Imgproc.cvtColor(source, tempMat, Imgproc.COLOR_RGB2GRAY);
        return tempMat;
    }

    Telemetry telemetry;


    public Mat highLightCorners(Mat hsvMat, int blocksizeInt, int harrisKSize, double harrisK) {
        Imgproc.cornerHarris(hsvMask, harrisMat, blocksizeInt, harrisKSize, harrisK);
        Core.normalize(harrisMat, harrisMat, 0, 255, Core.NORM_MINMAX);
        harrisMat.convertTo(harrisMat, CvType.CV_8U);
        return harrisMat;
    }

    @Override
    public void init(Mat firstFrame) {
        rgb2hsv(firstFrame);
    }

    public Mat blurMask(Mat input) {
        Size gaussainSize = new Size(size, size);
        Imgproc.GaussianBlur(input, tempMat, gaussainSize, 1.0);
        Core.inRange(tempMat, black, black, tempMat);
        Core.bitwise_not(tempMat, tempMat);
        return tempMat;
    }

    public Mat removeNoise(Mat input) {
        Mat morphKernal = new Mat(morph, morph, 0);
        tempMat = blurMask(input);
        Imgproc.morphologyEx(tempMat, tempMat, Imgproc.MORPH_OPEN, morphKernal);
        return tempMat;
    }

    public List<MatOfPoint> getContours(Mat input) {
        Mat hierarchy = new Mat();
        List<MatOfPoint> contoursArr = new ArrayList<>();
        Imgproc.findContours(input, contoursArr, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        return contoursArr;
    }

    public ArrayList<Point> getCenterPoint(List<MatOfPoint> contoursArr) {
        ArrayList<Point> output = new ArrayList<Point>();
        for (MatOfPoint contour : contoursArr) {
            Point point = new Point();
            Moments moments = Imgproc.moments(contour);
            //uses the moments to get the center of any object
            point.x = (int) moments.get_m10() / (int) moments.get_m00();
            point.y = (int) moments.get_m01() / (int) moments.get_m00();
            output.add(point);
        }
        return output;
    }

    public Mat drawRotatedRectange(Mat input, RotatedRect rotatedRect) {
        Point[] vertices = new Point[4];
        rotatedRect.points(vertices);

        // Draw the rotated rectangle on the image
        for (int i = 0; i < 4; i++) {
            Imgproc.line(input, vertices[i], vertices[(i + 1) % 4], new Scalar(0, 255, 0), 2);
        }
        return input;
    }

    public double getMaxRealX(){
        double distFrom = 1.0/Math.tan(Math.toRadians(fovDiag/2)) * Math.sqrt(Math.pow(width,2) +Math.pow(height,2));
        double x = (height)/(Math.tan(Math.toRadians(camAngleDeg)));
        double z = Math.tan(Math.toRadians(camAngleDeg)) * x - height;
        double maxX = Math.sqrt(Math.pow(x,2) + Math.pow(z+height, 2));
        return maxX;
    }

    public Point getRealPoint(Point camPoint){
        /*Parameters:
        x, y : float
        2D coordinates in the image plane (pixels).
        Z_real : float
            Real-world depth of the object (in camera's Z-up coordinate system).
        fx, fy : float
            Focal lengths of the camera in x and y directions (pixels).
        cx, cy : float
        Principal point offsets (pixels)
        */
        double fx = focalLen;
        double fy = focalLen;
        double cx = width/2;
        double cy = height/2;
        double z = ((1.0/Math.cos(Math.toRadians(camAngleDeg)) * zReal/(-fy *xConst4Cam * Math.tan(Math.toRadians(camAngleDeg))/(camPoint.y-cy)-1)));
        double X = -fy * z * xConst4Cam/(camPoint.y - cy);
        double Y = -(camPoint.x-cx) * z * yConst4Cam/(camPoint.y - cy);
        X = Math.sqrt(Math.pow(X,2)- Math.pow(zReal,2));
        return new Point(X, Y);


    }

    public List<MatOfPoint> simplifyContours(List<MatOfPoint> contours) {
        double epsilon;
        for (int i = 0; i < contours.size(); i++) {
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

    public ArrayList<Point> points2RealPoints(double realWidth, double realHeight, Point camPosRelative2TheCenterOfTheSubmersible, ArrayList<Point> points) {
        Point camPos = camPosRelative2TheCenterOfTheSubmersible;
        ArrayList<Point> newPoints = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            newPoints.add(new Point(points.get(i).x / width * realWidth - camPos.x, points.get(i).y / height * realHeight - camPos.y));
        }
        return newPoints;
    }

    public boolean inRange(double input, double min, double max) {
        return (input >= min && input <= max);
    }


    public int getNumberOfBlocks(MatOfPoint contour) {
        return (int) (Imgproc.contourArea(contour) / avgContourArea);
    }


    public MatOfPoint getClosestContour(Point startPoint,  List<MatOfPoint> contours){
        double dist = 999999;
        double minDist = 999999;
        Point bestPoint = new Point();
        MatOfPoint bestContour = new MatOfPoint();
        ArrayList<Point> output = new ArrayList<Point>();
        for (MatOfPoint contour : contours) {
            dist = Imgproc.pointPolygonTest(new MatOfPoint2f(contour.toArray()), startPoint, true);
            if (dist < minDist) {
                minDist = dist;
                bestContour = contour;
            }
        }
        return bestContour;
    }

    public void setChoice(int val){
        choice = val;
    }


    public Point getClosestPoint(Point startPoint, List<MatOfPoint> contours) {
        double dist = 999999;
        double minDist = 999999;
        Point bestPoint = new Point();
        MatOfPoint bestContour = new MatOfPoint();
        ArrayList<Point> output = new ArrayList<Point>();
        for (MatOfPoint contour : contours) {
            dist = Math.abs(Imgproc.pointPolygonTest(new MatOfPoint2f(contour.toArray()), startPoint, true));
            if (dist < minDist) {
                minDist = dist;
                bestContour = contour;
            }
        }
        dist = 99999;
        minDist = 99999;
        for (Point point : bestContour.toArray()) {
            dist = Math.sqrt(Math.pow(point.x, 2) + Math.pow(point.y, 2));
            if (dist < minDist) {
                minDist = dist;
                bestPoint = point;
            }
        }
        return bestPoint;
    }





    @Override
    public Mat processFrame(Mat input) {
        Imgproc.resize(input, input, new Size(width, height));

        Mat hsvMat = rgb2hsv(input);
        if (choice == 0) {
            Core.inRange(hsvMat, lowerRed, upperRed, hsvMask);
        } else if (choice == 1) {
            Core.inRange(hsvMat, lowerBlue, upperBlue, hsvMask);
        } else if (choice == 2) {
            Core.inRange(hsvMat, lowerYellow, upperYellow, hsvMask);
        } else {
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
        diluteKernal = new Mat(dilusionSize, dilusionSize, 0);
        Imgproc.erode(hsvMask, hsvMask, erodeKernal);
        Imgproc.dilate(hsvMask, hsvMask, diluteKernal);
        contours = getContours(hsvMask);
        closestPoint = getClosestPoint(startPoint, contours);
        epicContour = getClosestContour(startPoint ,contours);
        realPoint = getRealPoint(closestPoint);
        if(closestPoint.x != 0 ) {
            Imgproc.putText(input,
                    "x:" + realPoint.x +", y:" + realPoint.y,
                    new Point(10, 10),
                    1, 1, new Scalar(0, 0, 0), 2);
            Imgproc.circle(input, closestPoint, 3, new Scalar(255,0,0), 2);
            Imgproc.circle(input, startPoint, 3, new Scalar(255,0,255), 5);
        }
        return input;

    }
}
