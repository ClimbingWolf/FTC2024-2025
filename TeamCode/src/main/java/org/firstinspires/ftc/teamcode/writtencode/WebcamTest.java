package org.firstinspires.ftc.teamcode.writtencode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MaskByColor;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import com.acmerobotics.dashboard.FtcDashboard;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.reflect.Array;
import java.util.ArrayList;
import org.ejml.dense.row.FMatrixComponent;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

@TeleOp(name = "WebcamTest")
@Config
public class WebcamTest extends LinearOpMode {
    public ArrayList<Point> centerPoints;
    public double width = 320;
    public double height = 240;
    OpenCvWebcam webcam;
    MaskByColor pipeline;
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new MaskByColor();
        webcam.setPipeline(pipeline);

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(640,360, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });
        waitForStart();

        while (opModeIsActive()) {
            //centerPoints = pipeline.centerPoints;
        }

    }
    public ArrayList<Point> points2RealPoints(double realWidth, double realHeight, Point camPosRelative2TheCenterOfTheSubmersible, ArrayList<Point> points) {
        Point camPos = camPosRelative2TheCenterOfTheSubmersible;
        ArrayList<Point> newPoints = new ArrayList<>();
        for (int i = 0; i<points.size(); i++){
            newPoints.add(new Point(points.get(i).x/width * realWidth-camPos.x, points.get(i).y/height * realHeight-camPos.y));
        }
        return newPoints;
    }
}
