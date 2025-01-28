package org.firstinspires.ftc.teamcode.writtencode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.mathfunctions.FtcMath;
import org.opencv.core.Point;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;


@Config
@Disabled
public class SlidesController{
    public FtcDashboard dashboard = FtcDashboard.getInstance();
    public TelemetryPacket packet = new TelemetryPacket();
    public MotorEx leftSlide;

    public MotorEx rightSlide;

    public PIDFController pidfControllerLeft;
    public PIDFController pidfControllerRight;

    public static double kPL = 0;
    public static double kIL = 0;
    public static double kDL = 0;
    public static double kFL = 0;
    public static double kPR = 0;
    public static double kIR = 0;
    public static double kDR = 0;
    public static double kFR = 0;



    public double powerLeft = 0;
    public double powerRight = 0;
    public final double ticksPerInch = 3296;

    public static double multiplier =1;
    public SlidesController(MotorEx right, MotorEx left) {
        rightSlide = right;
        leftSlide = left;
        pidfControllerLeft = new PIDFController(kPL, kIL, kDL, kFL);
        pidfControllerRight = new PIDFController(kPR, kIR, kDR, kFR);
    }
    public SlidesController(MotorEx right, MotorEx left,double kPL, double kIL,double  kDL,double kFL,double kPR,double kIR,double kDR,double kFR) {
        rightSlide = right;
        leftSlide = left;
        pidfControllerLeft = new PIDFController(kPL, kIL, kDL, kFL);
        pidfControllerRight = new PIDFController(kPR, kIR, kDR, kFR);
    }

    public void setConstants(double kPL, double kIL,double  kDL,double kFL,double kPR,double kIR,double kDR,double kFR){
        pidfControllerLeft.setP(kPL);
        pidfControllerLeft.setI(kIL);
        pidfControllerLeft.setD(kDL);
        pidfControllerLeft.setF(kFL);
        pidfControllerRight.setP(kPR);
        pidfControllerRight.setI(kIR);
        pidfControllerRight.setD(kDR);
        pidfControllerRight.setF(kFR);
    }
    public void moveSlides(double inches){
        leftSlide.setTargetPosition((int)(inches * ticksPerInch));
        rightSlide.setTargetPosition((int)(inches * ticksPerInch));
        powerLeft = pidfControllerLeft.calculate(leftSlide.getCurrentPosition());
        powerRight = pidfControllerRight.calculate(rightSlide.getCurrentPosition());
        rightSlide.setVelocity(powerRight);
        leftSlide.setVelocity(powerLeft);
        packet.put("powerRight", powerRight);
        packet.put("powerLeft", powerLeft);
        dashboard.sendTelemetryPacket(packet);
    }



}
