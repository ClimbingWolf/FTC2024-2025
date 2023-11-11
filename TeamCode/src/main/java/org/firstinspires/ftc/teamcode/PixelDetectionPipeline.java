
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Core;
import org.opencv.core.Size;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

/*
 * This sample demonstrates a basic (but battle-tested and essentially
 * 100% accurate) method of detecting the skystone when lined up with
 * the sample regions over the first 3 stones.
 */

public class PixelDetectionPipeline extends OpenCvPipeline
{
    //constants
    public Mat tempIm = new Mat();
    private Scalar black = new Scalar(0,0,0);
    private Scalar yellowLower = new Scalar(17,80,201);
    private Scalar yellowUpper = new Scalar(40,255,255);
    private Scalar greenLower = new Scalar(42,52,82);
    private Scalar greenUpper = new Scalar(72,255,225);
    private Scalar whiteLower = new Scalar(0,0,192);
    private Scalar whiteUpper = new Scalar(255,15,255);
    private Scalar purpleLower = new Scalar(127,21,120);
    private Scalar purpleUpper = new Scalar(144,246,249);
    public static int size = 5;
    private Size gaussainSize = new Size(size, size);
    public static double gaussainSigma = 1.0;

    Mat inputToHsv(Mat input)
    {

        Imgproc.cvtColor(input, tempIm, Imgproc.COLOR_RGB2HSV);
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
        input = inputToHsv(input);
        Core.inRange(input, purpleLower, purpleUpper, tempIm);
        Core.bitwise_and(input, input, tempIm, tempIm);
        Imgproc.GaussianBlur(tempIm, tempIm, gaussainSize, 1.0);
        Core.inRange(tempIm, black, black, tempIm);
        Core.bitwise_not(tempIm, tempIm);

        return tempIm;
    }

    /*
     * Call this from the OpMode thread to obtain the latest analysis
     */
}
