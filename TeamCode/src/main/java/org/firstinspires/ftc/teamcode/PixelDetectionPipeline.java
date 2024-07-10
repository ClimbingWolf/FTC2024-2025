
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */

public class PixelDetectionPipeline extends OpenCvPipeline
{
    //constants
    private Mat tempIm = new Mat();

    private Mat purpleIm = new Mat();
    private Mat greenIm = new Mat();
    private Mat whiteIm = new Mat();
    private Mat yellowIm = new Mat();

    public static boolean green = false;

    public static boolean yellow = false;
    public static boolean purple = true;

    public static boolean white = true;

    private Scalar black = new Scalar(0,0,0);
    private Scalar skeleton = new Scalar(1,1,1);
    private Scalar yellowLower = new Scalar(17,80,201);
    private Scalar yellowUpper = new Scalar(40,255,255);
    private Scalar greenLower = new Scalar(42,52,82);
    private Scalar greenUpper = new Scalar(72,255,225);
    public static Scalar whiteLower = new Scalar(123,5,144);
    public static Scalar whiteUpper = new Scalar(180,50,195);
    private Scalar purpleLower = new Scalar(127,21,120);
    private Scalar purpleUpper = new Scalar(144,246,249);
    public static int size = 3;
    public static int morph = 3;
    //morph decides the size of the morphKernal
    public Mat morphKernal = new Mat(morph, morph, 0);
    private Size gaussainSize = new Size(size, size);
    public static double gaussainSigma = 1.0;

    Mat inputToHsv(Mat input)
    {
        Mat tempIm = new Mat();
        Imgproc.cvtColor(input, tempIm, Imgproc.COLOR_RGB2HSV);
        return tempIm;

    }

    ArrayList getCenterCords(Mat input){
        Mat tempIm = new Mat();
        Mat hierarchy = new Mat();
        Moments moments = new Moments();
        ArrayList<Point> outputArr = new ArrayList<>();
        Point point = new Point();
        //invert image to get shapes
        Core.bitwise_not(input, tempIm);
        List<MatOfPoint> contoursArr = new ArrayList<>();
        Imgproc.findContours(tempIm, contoursArr, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);
        for (int i = 0; i<contoursArr.size(); i++){
            moments = Imgproc.moments(contoursArr.get(i));
            //uses the moments to get the center of any object
            point.x = (int)moments.get_m10() / (int)moments.get_m00();
            point.y = (int)moments.get_m01() / (int)moments.get_m00();
            outputArr.add(point);
        }
        return outputArr;
    }

    Mat inputToRgb(Mat input){
        Mat tempIm = new Mat();
        Imgproc.cvtColor(input, tempIm, Imgproc.COLOR_HSV2RGB);
        return tempIm;
    }


    @Override
    public void init(Mat firstFrame)
    {
        inputToHsv(firstFrame);
    }

    @Override
    public Mat processFrame(Mat input)
    {
        Mat tempIm = new Mat();
        Mat combinedMask = new Mat();
        Core.inRange(input, whiteLower, whiteUpper, whiteIm);
        input = inputToHsv(input);
        Core.inRange(input, purpleLower, purpleUpper, purpleIm);
        Core.inRange(input, yellowLower, yellowUpper, yellowIm);
        Core.inRange(input, greenLower, greenUpper, greenIm);
        Core.inRange(input, whiteLower, whiteUpper, whiteIm);

        whiteIm = removeNoise(purpleIm);
        combinedMask = whiteIm;
        Core.bitwise_or(whiteIm, combinedMask, combinedMask);
        input = inputToRgb(input);
        Core.bitwise_and(input, input, tempIm, combinedMask);

        return tempIm;
    }

    public Mat removeNoise(Mat input){
        tempIm = blurMask(input);
        Imgproc.morphologyEx(tempIm, tempIm, Imgproc.MORPH_OPEN, morphKernal);
        return tempIm;
    }

    public Mat blurMask(Mat input){
        Imgproc.GaussianBlur(input, tempIm, gaussainSize, 1.0);
        Core.inRange(tempIm, black, black, tempIm);
        Core.bitwise_not(tempIm, tempIm);
        return tempIm;
    }

    public ArrayList findPositions(Mat input){
        return null;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
}
